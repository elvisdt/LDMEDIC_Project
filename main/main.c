#include <stdio.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <inttypes.h>

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>
#include <esp_event.h>

#include <esp_system.h>
#include <sdkconfig.h>
#include <sys/time.h>
#include <time.h>
#include <esp_timer.h>
#include <esp_log.h>
#include "esp_ota_ops.h"
#include <nvs.h>
#include <nvs_flash.h>

#include <cJSON.h>

/* main libraries */
#include "credentials.h"
#include "EG915_modem.h"
#include "ota_modem.h"
#include "main.h"
#include "inkbird_ble.h"

#include "modem_aux.h"

/* ota modem librarias */
#include "crc.h"
#include "ota_control.h"
#include "ota_esp32.h"
#include "ota_headers.h"

/* ota ble librarias */
// #include "gap.h"
// #include "gatt_svr.h"

/*------ble devices----*/
#include "esp_bt.h"
#include "esp_gap_ble_api.h"
#include "esp_gattc_api.h"
#include "esp_gatt_defs.h"
#include "esp_bt_main.h"
#include "esp_gatt_common_api.h"

/***********************************************
 * DEFINES
************************************************/

#define TAG          "MAIN"
#define MAX_ATTEMPS     3

#define WAIT_MS(x)		vTaskDelay(pdMS_TO_TICKS(x))
#define WAIT_S(x)		vTaskDelay(pdMS_TO_TICKS(x*1e3))

#define MASTER_TOPIC_MQTT   "LDMEDIC"


#define	INIT_DELAY_BLE		3

#define NAMESPACE_NVS		"storage"
#define KEY_BLE_LIST		"BLE_LIST"
#define KEY_IP_MQTT			"IP_MQTT"
#define KEY_BLE_TIME		"BLET"


#define PROFILE_A_APP_ID 	0
#define SCAN_DURATION       30 

/*----> >-----*/
#define     PIN_CIRCULINA      GPIO_NUM_11

/***********************************************
 * STRUCTURES
************************************************/
static esp_ble_scan_params_t ble_scan_params = {
    // Inicializa aquí los parámetros de escaneo BLE
    .scan_type              = BLE_SCAN_TYPE_ACTIVE,
    .own_addr_type          = BLE_ADDR_TYPE_PUBLIC,
    .scan_filter_policy     = BLE_SCAN_FILTER_ALLOW_ALL,
    .scan_interval          = 0x0100,//100ms , range  0x0004 to 0x4000
    .scan_window            = 0x0050,//50ms , range  0x0004 to 0x4000
    .scan_duplicate         = BLE_SCAN_DUPLICATE_ENABLE
};


/***********************************************
 * VARIABLES
************************************************/
/*---> <---*/
QueueHandle_t AlertQueue;

/*---> External variables <--*/
QueueHandle_t uart_modem_queue;
uint8_t rx_modem_ready;
int rxBytesModem;
uint8_t *p_RxModem;
int end_task_uart_m95;


/*---> Task Handle <---*/
TaskHandle_t UART_task_handle    	=   NULL;
TaskHandle_t MAIN_task_handle    	=   NULL;
TaskHandle_t BLE_Task_handle        =   NULL;
TaskHandle_t ALARM_Task_handle      =   NULL;

/*---> Data OTA Output <---*/
cJSON *doc;
char * output;


/*---> Aux Mememory <---*/
uint8_t aux_buff_mem[BUF_SIZE_MODEM];
char* buff_aux=(char*)aux_buff_mem;


/*---> data structure for modem <---*/
static modem_gsm_t data_modem={0};
static int ret_update_time =0;


/*---> gpio and uart config <---*/
EG915_gpio_t modem_gpio;
EG915_uart_t modem_uart;


/*---> OTA <--*/
uint8_t watchdog_en=1;
uint32_t current_time=0;


/*--> MQTT CHARS <---*/
int mqtt_idx = 0; 		// 0->5


/*--> ble varibles<--*/
time_t unix_time=0;
static bool scanning = false;
static size_t sen_ble_count = 0;

uint16_t delay_ble=INIT_DELAY_BLE;
size_t BLE_size  = sizeof(ink_list_ble_addr_t);
static ink_list_ble_addr_t list_ble_device={0};
ink_list_ble_data_t list_ble_data={0};


/*--> global main variables <--*/
uint8_t     delay_tmax = 10;
uint8_t     delay_tmin = 5;

uint8_t     delay_info = 5;
uint32_t    Info_time=0;
uint32_t    MQTT_read_time=0;
uint32_t    OTA_md_time=1;
uint32_t    SMS_time=0;

/*--> NVS <---*/
nvs_handle_t storage_nvs_handle;

//size_t BLE_size  = sizeof(ink_list_ble_addr_t);
size_t MQTT_size = 50;
char ip_mqtt_connect[50] = ip_MQTT;

/***********************************************
 * FUNCIONTIONS
************************************************/
void BLE_init(void);
void BLE_scanner_start(void);
void BLE_scanner_stop(void);
static void esp_gap_cb(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t* param);


void main_cfg_parms(){

    // ---> placa negra GPS <--- //
	modem_gpio.gpio_reset  = GPIO_NUM_35;
    modem_gpio.gpio_pwrkey = GPIO_NUM_48;
    modem_gpio.gpio_status = GPIO_NUM_42;

    modem_uart.uart_num     = UART_NUM_2;
    modem_uart.baud_rate    = 115200;
    modem_uart.gpio_uart_rx = GPIO_NUM_36;
    modem_uart.gpio_uart_tx = GPIO_NUM_37;
    
    //------placa morada cartavio-------------//
    /*
	modem_gpio.gpio_reset   = GPIO_NUM_1;
    modem_gpio.gpio_pwrkey  = GPIO_NUM_2;
    modem_gpio.gpio_status  = GPIO_NUM_40;

    modem_uart.uart_num     = UART_NUM_2;
    modem_uart.baud_rate    = 115200;
    modem_uart.gpio_uart_rx = GPIO_NUM_41;
    modem_uart.gpio_uart_tx = GPIO_NUM_42;
    */

    ESP_ERROR_CHECK(gpio_reset_pin(PIN_CIRCULINA));
    ESP_ERROR_CHECK(gpio_set_direction(PIN_CIRCULINA, GPIO_MODE_OUTPUT));
    gpio_set_level(PIN_CIRCULINA,0); // APAGAR CIRCULNA

};

/**
 * Inicializa la configuración del módem, encendiéndolo y ejecutando los comandos de inicio.
 * 
 * @return MD_CFG_SUCCESS si la configuración se inicializa correctamente, MD_CFG_FAIL en caso contrario.
 */
int Init_config_modem(){
    ESP_LOGW(TAG, "--> INIT CONFIG MODEM <--");
    int state = 0;
    for (size_t i = 0; i < MAX_ATTEMPS; i++){
        state  = Modem_turn_ON();
        if (state !=MD_CFG_SUCCESS){
            Modem_turn_OFF();
            WAIT_S(3);
        }else{
            break;
        }
    }

    if(state != MD_CFG_SUCCESS) return state;

    for (size_t i = 0; i < MAX_ATTEMPS; i++){
        WAIT_S(2);
        state  = Modem_begin_commands();
        if (state ==MD_AT_OK){
            return MD_CFG_SUCCESS;
        }
    }
	return MD_CFG_FAIL;
}

/**
 * Activa el módem realizando la inicialización de la configuración y obteniendo la información del dispositivo.
 * 
 * @return MD_CFG_SUCCESS si el módem se activa correctamente, MD_CFG_FAIL en caso contrario.
 */
int Active_modem(){
	int status = Init_config_modem();
	if (status == MD_CFG_SUCCESS){
        int status = Modem_get_dev_info(&data_modem.info);
        if(status ==MD_CFG_SUCCESS){
            return MD_CFG_SUCCESS;   // FAIL
        }
    }
	return MD_CFG_FAIL;
}

/* ---- INIT NVS ---*/

void Init_NVS_Keys(){
	/*------------------------------------------------*/
	ESP_LOGI(TAG,"Init NVS keys of data");

	esp_err_t err = nvs_flash_init();
	if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
		ESP_ERROR_CHECK(nvs_flash_erase());
		err = nvs_flash_init();
	}
    ESP_ERROR_CHECK(err);
	

	ESP_LOGI(TAG,"NVS get WMBUS keys");

    
	nvs_open(NAMESPACE_NVS,NVS_READWRITE,&storage_nvs_handle);

	// Recuperar estructura ink_list_ble_addr_t
	err = nvs_get_blob(storage_nvs_handle, KEY_BLE_LIST, &list_ble_device, &BLE_size);
	if(err == ESP_ERR_NVS_NOT_FOUND){
		ink_init_list_ble_addr(&list_ble_device);
		nvs_set_blob(storage_nvs_handle, KEY_BLE_LIST, &list_ble_device, sizeof(ink_list_ble_addr_t));
	}
	err=nvs_get_u16(storage_nvs_handle,KEY_BLE_TIME,&delay_ble);
	if (err == ESP_ERR_NVS_NOT_FOUND) {
		delay_ble=INIT_DELAY_BLE;
		nvs_set_u16(storage_nvs_handle,KEY_BLE_TIME,delay_ble);
	}

	err=nvs_get_str(storage_nvs_handle,KEY_IP_MQTT,ip_mqtt_connect,&MQTT_size);
	if (err == ESP_ERR_NVS_NOT_FOUND){
		strcpy(ip_mqtt_connect,ip_MQTT);
		nvs_set_str(storage_nvs_handle,KEY_IP_MQTT,ip_mqtt_connect);
	}
	
	ink_concat_lit_ble(list_ble_device,buff_aux);
	printf("------------------------\r\n");

	printf("->MQTT:\r\n"
		   "ip->\"%s\"\r\n"
		   "\r\n",
		   ip_mqtt_connect);
	
	printf("->BLE:\r\n"
		  "Interval: %u min\r\n"
		  "List:\r\n"
		  "%s"
		  "\r\n",
		  delay_ble,buff_aux);

	printf("------------------------\r\n");
    
	//-------------------------------------------------//
	ESP_LOGI(TAG,"NVS keys recovered succsesfull\r\n");
	return;
}

/**
 * @brief Checks for OTA (Over-The-Air) updates.
 *
 * This function establishes a connection to an OTA server, sends device information,
 * and waits for a response. If an OTA update is pending (indicated by the response),
 * it initiates the OTA process.
 *
 * @note The specific details such as 'ip_OTA', 'port_OTA', and custom functions are
 * application-specific and should be defined elsewhere in your code.
 */
void OTA_Modem_Check(void){

    static const char *TAG_OTA = "OTA_MD";
    esp_log_level_set(TAG_OTA, ESP_LOG_INFO);

    ESP_LOGI(TAG_OTA,"==MODEM OTA CHECK ==");
    char buffer[500] ="";
    do{
        if(TCP_open(ip_OTA, port_OTA)!=MD_TCP_OPEN_OK){
            ESP_LOGW(TAG_OTA,"Not connect to the Server");
            TCP_close();
            break;
        }
        ESP_LOGI(TAG_OTA,"Requesting update...");
        if(TCP_send(output, strlen(output))==MD_TCP_SEND_OK){                           // 1. Se envia la info del dispositivo
            ESP_LOGI(TAG_OTA,"Waiting for response...");
            readAT("}\r\n", "-8/()/(\r\n",10000,buffer);   // 2. Se recibe la 1ra respuesta con ota True si tiene un ota pendiente... (el servidor lo envia justo despues de recibir la info)(}\r\n para saber cuando llego la respuesta)
            debug_ota("main> repta %s\r\n", buffer);
            if(strstr(buffer,"\"ota\": \"true\"") != 0x00){
                ESP_LOGI(TAG_OTA,"Start OTA download");
                ESP_LOGW(TAG_OTA,"WDT deactivate");
                watchdog_en=0;
                if(Ota_UartControl_Modem() == OTA_EX_OK){
                    ESP_LOGI(TAG_OTA,"OTA UPDATE SUCCESFULL, RESTART");
                    vTaskDelay(pdMS_TO_TICKS(1000));
                    esp_restart();
                }else{
                    ESP_LOGW(TAG_OTA,"FAIL OTA UPDATE");
                }
                current_time = pdTICKS_TO_MS(xTaskGetTickCount())/1000;
                watchdog_en=1;
                ESP_LOGW(TAG_OTA,"WDT reactivate");
            }
            int ret_tcp=TCP_close(); // close tcp
            printf("ret : 0x%x\r\n",ret_tcp);
        }
    }while(false);
    return;
}

/***********************************************
 * MODEM UART TASK
************************************************/

static void Modem_rx_task(void *pvParameters){
    uart_event_t event;
    uint8_t* dtmp = (uint8_t*) malloc(RD_BUF_SIZE);
    p_RxModem = dtmp;
    for(;;) {
        if(xQueueReceive(uart_modem_queue, (void * )&event, portMAX_DELAY)) {
        	bzero(dtmp, RD_BUF_SIZE);
            if(event.type == UART_DATA) {
				rxBytesModem=event.size;
				uart_read_bytes(modem_uart.uart_num, dtmp, event.size, portMAX_DELAY*2);
				p_RxModem=dtmp;
				rx_modem_ready=1;
            }
        }
        vTaskDelay(100/portTICK_PERIOD_MS);
    }
    free(dtmp);
    dtmp = NULL;
    vTaskDelete(NULL);
}


int CheckRecMqtt(void){
    ESP_LOGI("MAIN-MQTT","--> Revisando conexion <--");
    static int num_max_check = 0;
    static int ret_conn=0;
    static int ret_open=0;

    if (num_max_check >=(MAX_ATTEMPS+2)){
        ESP_LOGE("MAIN-MQTT", "RESTART ESP32");
        esp_restart();
    }
    printf("num_max: %d\r\n", num_max_check);

    ret_conn = Modem_Mqtt_CheckConn(mqtt_idx);
    printf("ret_conn: 0x%X\r\n", ret_conn);
    if (ret_conn == MD_MQTT_CONN_ERROR){
        ret_open=Modem_Mqtt_CheckOpen(mqtt_idx,ip_mqtt_connect, port_MQTT);
        printf("ret_open: 0x%X\r\n",ret_open);
        if (ret_open == MD_CFG_FAIL){
            num_max_check ++;
        }else if (ret_open==MD_MQTT_IS_OPEN){
            //Conectamos son nuesto indice e imei
            Modem_Mqtt_Conn(mqtt_idx, data_modem.info.imei);
        }else if (ret_open==MD_MQTT_NOT_OPEN){
            num_max_check ++;
            // Configurar y Abrir  MQTT
            ret_open=Modem_Mqtt_Open(mqtt_idx,ip_mqtt_connect,port_MQTT);
            if (ret_open == MD_MQTT_OPEN_OK){
                // Abrir comunicacion  y suscripcion
                Modem_Mqtt_Conn(mqtt_idx, data_modem.info.imei);
            }else{
                WAIT_S(2);
                // desconectar y cerrar en caso exista alguna comunicacion
                Modem_Mqtt_Disconnect(mqtt_idx);
                Modem_Mqtt_Close(mqtt_idx);
            }
        }
        CheckRecMqtt(); // Volvemos a verificar la conexion
    }else if (ret_conn==MD_MQTT_CONN_INIT || ret_conn == MD_MQTT_CONN_DISCONNECT){
        WAIT_S(1);
        Modem_Mqtt_Conn(mqtt_idx, data_modem.info.imei);
        CheckRecMqtt();  // CHECK AGAIN
    }else if (ret_conn==MD_MQTT_CONN_CONNECT){
        WAIT_S(1);
        CheckRecMqtt();  // CHECK AGAIN
    }else if (ret_conn==MD_MQTT_CONN_OK){
        num_max_check = 0;
        ESP_LOGI("MAIN-MQTT","CONNECT SUCCESFULL");
        // printf("TOPIC SUB: \'%s\'\r\n",topic_sub);
        // printf("ret_sub: 0x%X\r\n",state_mqtt_sub);
    }
	return ret_conn;
}

/************************************************
 * BLE CONTROLLERS (FUNCS AND TASK)
*************************************************/


// STATIC
static void esp_gap_cb(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t* param) {

    // uint8_t *adv_name = NULL;
    // uint8_t adv_name_len = 0;
    uint8_t *manufacturer_data;
    uint8_t adv_manufacturer_len = 0;

	esp_bd_addr_t addr_scan={0};

    switch (event) {
    case ESP_GAP_BLE_SCAN_RESULT_EVT: {
        esp_ble_gap_cb_param_t *scan_result = (esp_ble_gap_cb_param_t *)param;
        switch (scan_result->scan_rst.search_evt) {
			case ESP_GAP_SEARCH_INQ_RES_EVT:{
				// deshabilitar si se completa los dispositivos máximos
				if (sen_ble_count >= MAX_BLE_DEVICES || list_ble_device.num_ble==0) {
					BLE_scanner_stop();
					scanning =false;
					break;
				}
				// adv_name = esp_ble_resolve_adv_data(scan_result->scan_rst.ble_adv, ESP_BLE_AD_TYPE_NAME_CMPL,&adv_name_len);
				//manufacturer_data = esp_ble_resolve_adv_data(scan_result->scan_rst.ble_adv,	ESP_BLE_AD_MANUFACTURER_SPECIFIC_TYPE,&adv_manufacturer_len);
				esp_log_buffer_hex(TAG, scan_result->scan_rst.bda, 6);
				memcpy(addr_scan, scan_result->scan_rst.bda, 6);
				//ESP_LOGW(TAG, "searched MANUFACTURE Len %d", adv_manufacturer_len);

				
				int idx = -1;
				if (ink_check_addr_scan(addr_scan,&list_ble_device,&idx)==1){
					manufacturer_data = esp_ble_resolve_adv_data(scan_result->scan_rst.ble_adv,
												ESP_BLE_AD_MANUFACTURER_SPECIFIC_TYPE,
												&adv_manufacturer_len);
					ESP_LOGW(TAG, "searched MANUFACTURE Len %d", adv_manufacturer_len);

					if(adv_manufacturer_len>1){
						strcpy(list_ble_data.data_ble[sen_ble_count].name, list_ble_device.addr_scan[idx].name);// copiando el nombre
						memcpy(list_ble_data.data_ble[sen_ble_count].addr, list_ble_device.addr_scan[idx].addr, 6);
						memcpy(list_ble_data.data_ble[sen_ble_count].data_Hx, manufacturer_data, adv_manufacturer_len);
                        list_ble_data.data_ble[sen_ble_count].cfg_tem = list_ble_device.addr_scan[idx].cfg_tem;

						time(&unix_time);
						list_ble_data.data_ble[sen_ble_count].epoch_ts = unix_time;
						sen_ble_count++;
						list_ble_data.num_ble =sen_ble_count;
                        
					}else{
						list_ble_device.addr_scan[idx].scan_ok = false;
					}
				}
				break;
				}
			case ESP_GAP_SEARCH_INQ_CMPL_EVT:{
				ESP_LOGI("BLE", "End scan...\n");
				scanning = false;
				break;
				}
			default:
				break;
        }
        break;
        }
    case ESP_GAP_BLE_SCAN_STOP_COMPLETE_EVT: {
        scanning = false;
        if (param->scan_stop_cmpl.status != ESP_BT_STATUS_SUCCESS) {
            ESP_LOGE(TAG, "Scan stop failed, error status = %x", param->scan_stop_cmpl.status);
        }
        break;
        }
    default:
        break;
    }
}


void BLE_init(void){
	ESP_LOGI(TAG,"-------BLE INIT----");
    esp_err_t err;
	ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT));
	esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
	err = esp_bt_controller_init(&bt_cfg);
    if (err) {
        ESP_LOGE(TAG, "%s initialize controller failed: %s", __func__, esp_err_to_name(err));
    }
	
    err = esp_bt_controller_enable(ESP_BT_MODE_BLE);
    if (err) {
        ESP_LOGE(TAG, "%s enable controller failed: %s", __func__, esp_err_to_name(err));
    }

    // Initialize Bluedroid with configuration
    esp_bluedroid_config_t bluedroid_cfg = {
        .ssp_en = true  // Habilita el emparejamiento seguro simple (SSP)
    };

    err = esp_bluedroid_init_with_cfg(&bluedroid_cfg);
    if (err) {
        ESP_LOGE(TAG, "%s init bluetooth failed: %s", __func__, esp_err_to_name(err));
    }

    err = esp_bluedroid_enable();
    if (err) {
        ESP_LOGE(TAG, "%s enable bluetooth failed: %s", __func__, esp_err_to_name(err));
    }

    //register the  callback function to the gap module
	err = esp_ble_gap_register_callback(esp_gap_cb);
    if (err){
        ESP_LOGE(TAG, "%s gap register failed, error code = %x", __func__, err);    
	}
	

    err = esp_ble_gattc_app_register(PROFILE_A_APP_ID);
    if (err){
        ESP_LOGE(TAG, "%s gattc app register failed, error code = %x", __func__, err);
    }

	err = esp_ble_gatt_set_local_mtu(500);
    if (err){
        ESP_LOGE(TAG, "set local  MTU failed, error code = %x",err);
    }
	return;
}

void BLE_scanner_start(void) {
    if (!scanning) {
        esp_ble_gap_set_scan_params(&ble_scan_params); 
        esp_ble_gap_start_scanning(SCAN_DURATION);
        scanning = true;
        printf("Started BLE scanning...\n\r");
    }
}

void BLE_scanner_stop(void) {
    if (scanning) {
        esp_ble_gap_stop_scanning();
        scanning = false;
        ESP_LOGI(TAG, "Stopped BLE scanning");
    }
}

/*--- BLE OTA INIT---*/
/*
void init_ble_ota() {
    ESP_LOGI(TAG, " BLE OTA INIT \r\n");
    nimble_port_init();
    ble_hs_cfg.sync_cb = sync_cb;
    ble_hs_cfg.reset_cb = reset_cb;
    gatt_svr_init();
    ble_svc_gap_device_name_set(device_name);
    nimble_port_freertos_init(host_task);
}
*/

bool run_diagnostics() {
    // Verificar si se realizó una actualización OTA correctamente
    // Si no se detectó una actualización OTA, simplemente retornar verdadero
    return true;
}

/**********************************************
 * MAIN TASK CONTROL AND FUNCS
***********************************************/
void Info_Send(void){
    ESP_LOGI("MQTT-INFO","<-- Send device info -->");


	static char topic[60]="";
	sprintf(topic,"%s/%s/INFO",MASTER_TOPIC_MQTT,data_modem.info.imei);

	time(&data_modem.time);
	data_modem.signal = Modem_get_signal();
	
	js_modem_to_str(data_modem, list_ble_device.num_ble, buff_aux);
	int ret_check =  CheckRecMqtt();
    ESP_LOGI("MQTT-INFO","ret-conn: 0x%X",ret_check);
	if(ret_check ==MD_MQTT_CONN_OK){
		ret_check = Modem_Mqtt_Pub(buff_aux,topic,strlen(buff_aux),mqtt_idx, 0);
        ESP_LOGI("MQTT-INFO","ret-pubb: 0x%X",ret_check);
        WAIT_S(1);
	}
	// Modem_Mqtt_Disconnect(mqtt_idx);
    return;
}

void BLE_send(void){
	int status = 0;
	int data_len=0;
	char topico[100]={0};
	
	ESP_LOGI("BLE-SEND","Enviando data por MQTT... \r\n");
	char mac_aux[20];
	status = CheckRecMqtt();
	if(status ==MD_MQTT_CONN_OK){
		//M95_PubMqtt_data(aux_buff,topico,strlen(aux_buff),tcpconnectID);
        int alert_ble = ALERT_BLE_DEACTIVE;
		for (size_t i = 0; i < list_ble_data.num_ble; i++){
			ink_esp_bd_addr__to__string_1(list_ble_data.data_ble[i].addr,mac_aux);
			sprintf(topico,"%s/%s/%s",MASTER_TOPIC_MQTT,data_modem.info.imei,mac_aux);
			js_record_data_ble(list_ble_data.data_ble[i], buff_aux);
			data_len = strlen(buff_aux);
			Modem_Mqtt_Pub(buff_aux,topico,data_len,mqtt_idx, 0);
			vTaskDelay(pdMS_TO_TICKS(1000));
            if (alert_ble == ALERT_BLE_DEACTIVE){
                alert_ble = ink_check_limit_temp(list_ble_data.data_ble[i]);
            }
		}
        if ( alert_ble != ALERT_BLE_DEACTIVE){
            // enviar alerta
            ESP_LOGE("BLE-SEND","SEND ALERT TO TASK CIRCULINA");
            xQueueSend(AlertQueue, &alert_ble, portMAX_DELAY); 
        }
        
	}
	list_ble_data.data_ready=false;
	list_ble_data.num_ble = 0;
	printf("BLE: Data de sensor enviada \r\n");
}

/**
 * Lee los datos de un mensaje MQTT y los procesa.
 */
void MQTT_Read(void){
    ESP_LOGI("MQTT-READ","<-- READ MQTT DATA -->");

    int state_mqtt_sub=-0x01;
    static char topic_sub[60]={0};
    sprintf(topic_sub,"%s/%s/CONFIG",MASTER_TOPIC_MQTT, data_modem.info.imei);

    state_mqtt_sub = Modem_Mqtt_Sub_Topic(mqtt_idx, topic_sub, buff_aux);
    printf("sub mqtt= 0x%X\r\n",state_mqtt_sub);
    if (state_mqtt_sub == MD_CFG_SUCCESS) {
        printf("DATA: %s\r\n",buff_aux);
        sprintf(topic_sub,"%s/%s/AWS",MASTER_TOPIC_MQTT, data_modem.info.imei);
        //Modem_Mqtt_Pub(buff_aux,topic_sub,strlen(buff_aux),mqtt_idx,0);
    }
    
    Modem_Mqtt_Unsub(mqtt_idx, topic_sub);
    return;
}

void SMS_check(void){

    const char* TAG_SMS = "SMS";
    ESP_LOGI(TAG_SMS,"----> CHECK SMS <----");

	int ret_sms = 0;
	uint8_t sms_tmp[1024];
	char * message = (char *)sms_tmp;

	char phone[20]={0};
	char enviar_sms=0;
	
	ret_sms=Modem_SMS_Read(message, phone);
    printf("ret_sms_chek = 0x%X\r\n",ret_sms);

	if (ret_sms==MD_SMS_READ_FOUND){
		remove_spaces(message);	   // eliminar espacios o saltos de linea
		str_to_uppercase(message); // convertir todos los mesajes a mayusculas
		ESP_LOGI(TAG_SMS,"->%s",message);
        if (strstr(message,"BLE")!=NULL){
			enviar_sms=1;
			if(strstr(message,"BLE,T,")!=NULL){
                int delay_aux =extraer_numero(message);
				if (delay_aux>1){
					delay_ble = delay_aux;
					ESP_LOGW(TAG_SMS,"NEW TIME SUCCESFULL: %d",delay_ble);
				}else{
					ESP_LOGW(TAG_SMS,"FAIL UPDATE SLEEP TIME");
				}
			}else if(strstr(message,"BLE,A,")!=NULL){
                char mac_str[18];   // MAC (formato XX:XX:XX:XX:XX)
                char nombre[10];	// nombre
                ret_sms=extraer_mac_y_nombre(message,mac_str,nombre);
                ESP_LOGI(TAG, "SMS extrac mac name: %d",ret_sms);
                if (ret_sms==1)	{
                    esp_bd_addr_t addr_aux;
                    // se convierte el mac en ADDR
                    ret_sms= ink_string__to__esp_bd_addr(mac_str,&addr_aux);
                    ESP_LOGI(TAG, "SMS mac err: %d",ret_sms);
                    if (ret_sms==1){
                        ret_sms=ink_check_exist_ble(addr_aux,list_ble_device);
                        int num_ble_reg = list_ble_device.num_ble;
                        if ( ret_sms!=1 && num_ble_reg<MAX_BLE_DEVICES){
                            memcpy(list_ble_device.addr_scan[num_ble_reg].addr,addr_aux, 6);
                            strcpy(list_ble_device.addr_scan[num_ble_reg].name,nombre);
                            list_ble_device.num_ble = num_ble_reg +1;
                        }else{
                            ESP_LOGW(TAG_SMS,"BLE ADD FAIL- DEVICE EXIST");
                        }
                    }else{
                        ESP_LOGE(TAG_SMS,"BLE ADD FAIL");
                    }
                }else{
                    ESP_LOGE(TAG_SMS,"MAC IS INCORRECT");
                }   
			}else if(strstr(message,"BLE,C,")!=NULL){
                char mac_str[18];   // MAC (formato XX:XX:XX:XX:XX)
                float tmax;
                float tmin;
                ret_sms=extraer_mac_tmax_tmin(message,mac_str,&tmax,&tmin);
                ESP_LOGI(TAG, "SMS extrac mac name: %d",ret_sms);
                if (ret_sms==1)	{
                    esp_bd_addr_t addr_aux;
                    // se convierte el mac en ADDR
                    ret_sms= ink_string__to__esp_bd_addr(mac_str,&addr_aux);
                    ESP_LOGI(TAG, "SMS mac err: %d",ret_sms);
                    if (ret_sms==1){
                        ret_sms=ink_check_exist_ble(addr_aux,list_ble_device);
                        ESP_LOGI(TAG, "exist err: %d",ret_sms);
                        int num_ble_reg = list_ble_device.num_ble;
                        if (ret_sms==1){
                            ink_cfg_tem_t cfg_tem_aux={1,tmax,tmin};
                            list_ble_device.addr_scan[num_ble_reg-1].cfg_tem = cfg_tem_aux;
                             ESP_LOGI(TAG_SMS,"BLE CFG SUCCESFULL");
                        }else{
                            ESP_LOGW(TAG_SMS,"BLE CFG FAIL- DEVICE NO EXIST");
                        }
                    }else{
                        ESP_LOGE(TAG_SMS,"BLE ADD FAIL");
                    }
                }else{
                    ESP_LOGE(TAG_SMS,"MAC IS INCORRECT");
                }  
			}else if(strstr(message,"BLE,E")!=NULL){
				ink_init_list_ble_addr(&list_ble_device);
				ESP_LOGW(TAG_SMS,"lista BLE eliminada");

			}
		}else if(strstr(message,"INFO")!=NULL){
			enviar_sms=1;
			ESP_LOGI(TAG_SMS,"Informacion solicitada");
		}else if(strstr(message,"RESET")!=NULL){
			ESP_LOGE(TAG_SMS,"Reset solicitado");
			Modem_turn_OFF_command();
			esp_restart();
		}else if(strstr(message,"MQTT,")!=NULL){
			enviar_sms=1;
			char ip_aux[50];
			int rr= split_and_check_IP(message,ip_aux);
			if (rr==1){
				strcpy(ip_mqtt_connect,ip_aux);
                Modem_Mqtt_Disconnect(mqtt_idx);
                WAIT_S(1);
			}
		}else{
			ESP_LOGW(TAG_SMS,"SMS No valido");
		}

		/*===================================================*/
        
		nvs_erase_key(storage_nvs_handle,KEY_BLE_TIME);
		nvs_erase_key(storage_nvs_handle,KEY_BLE_LIST);
		nvs_erase_key(storage_nvs_handle,KEY_IP_MQTT);

		nvs_set_u16(storage_nvs_handle,KEY_BLE_TIME,delay_ble);
		nvs_set_blob(storage_nvs_handle, KEY_BLE_LIST, &list_ble_device, sizeof(ink_list_ble_addr_t));
		nvs_set_str(storage_nvs_handle,KEY_IP_MQTT,ip_mqtt_connect);

		nvs_commit(storage_nvs_handle); // guardar cambios de maenra permanente
		/*===================================================*/	
	}

	if (enviar_sms){
		enviar_sms=0;
		memset(message, '\0',strlen(message));
		ink_concat_lit_ble(list_ble_device,buff_aux);

		sprintf(message,"IMEI: %s\n"
						"ip_mqtt: %s\n"
						"code: %s\n"
						"\n"
						"->BLE-<\n"
						"Interval: %u min\n"
						"List:\n"
						"%s",
						data_modem.info.imei, ip_mqtt_connect, PROJECT_VER, delay_ble,buff_aux);
		ret_sms = Modem_SMS_Send(message, phone);
		vTaskDelay(pdMS_TO_TICKS(5000));
		ESP_LOGI(TAG_SMS, "send sms status :0x%X",ret_sms);
		printf("phone:%s\r\n",phone);
		printf("send:\r\n%s\r\n",message);
	}

    ret_sms =Modem_SMS_delete();
    ESP_LOGI(TAG_SMS, "delete sms :0x%X",ret_sms);
    return;
}

/**********************************************
 * BLE SCAN TASK
**********************************************/

static void BLE_scann_task(void *pvParameters){
	ESP_LOGI("BLE","---scan task---.");
	time_t time_modem;
    time(&time_modem);
    time_t time_scan  = time_modem-(delay_ble*45);
	WAIT_S(2);

	for(;;){
		time(&time_modem);
		if (difftime(time_modem,time_scan)>(delay_ble*60)){
			time_scan=time_modem;
			if (list_ble_device.num_ble>0){
				// inicializar parametros
				sen_ble_count=0;
				for (size_t i = 0; i < list_ble_device.num_ble; i++) {
					list_ble_device.addr_scan[i].scan_ok = false;
				}

				ink_print_list_ble_addr(list_ble_device);
				BLE_scanner_start();
				vTaskDelay(pdMS_TO_TICKS(1000));
				do{
					vTaskDelay(pdMS_TO_TICKS(1000));
				}while (scanning);

				if (list_ble_data.num_ble>0){
					list_ble_data.data_ready = true;
				}
				ESP_LOGI(TAG, "NUM LIST DATA : %d",list_ble_data.num_ble);
			}else{
				ESP_LOGI("BLE","MAC NOT REGISTERS");
			}
		}

		vTaskDelay(pdMS_TO_TICKS(1000));
	}
	vTaskDelete(NULL);
}
/**********************************************
 * CIRCULINA TASK
**********************************************/
void AlarmTask(void *pvParameters) {
    int alert_value;
    while (1) {

        if (xQueueReceive(AlertQueue, &alert_value, portMAX_DELAY) == pdTRUE) {
            // Se recibió un valor de sensor fuera de límites
            if (alert_value == ALERT_BLE_TMAX) {
                gpio_set_level(PIN_CIRCULINA,1);
                WAIT_S(delay_tmax);
                gpio_set_level(PIN_CIRCULINA,0);

            } else if (alert_value == ALERT_BLE_TMIN) {
                gpio_set_level(PIN_CIRCULINA,1);
                WAIT_S(delay_tmin);
                gpio_set_level(PIN_CIRCULINA,0);
            }
        }
    }
}


/**********************************************
 * MAIN TASK
**********************************************/
static void Main_Task(void* pvParameters){
    WAIT_S(3);
	for(;;){
        
		current_time=pdTICKS_TO_MS(xTaskGetTickCount())/1000;
		if (current_time%30==0){
			printf("Tiempo: %lu\r\n",current_time);
		}

        // SEND INFO DATA
		if ((pdTICKS_TO_MS(xTaskGetTickCount())/1000) >= Info_time){
			current_time=pdTICKS_TO_MS(xTaskGetTickCount())/1000;
			Info_time+= delay_info*60;// cada 1 min
			if(ret_update_time!=MD_CFG_SUCCESS){
			    ret_update_time=Modem_update_time(1);
			}
            Info_Send();
            WAIT_S(1);
		}

        if (list_ble_data.data_ready){
			current_time=pdTICKS_TO_MS(xTaskGetTickCount())/1000;
			BLE_send();
			printf("MQTT BLE tomo %lu segundos\r\n",(pdTICKS_TO_MS(xTaskGetTickCount())/1000-current_time));
			vTaskDelay(100);
		}

        // SEND CHECK READ DATA
        if ((pdTICKS_TO_MS(xTaskGetTickCount())/1000) >= MQTT_read_time){
			current_time=pdTICKS_TO_MS(xTaskGetTickCount())/1000;
			MQTT_read_time+= 10;// cada 20 seg
            // MQTT_Read();
            WAIT_S(1);
		}
        if ((pdTICKS_TO_MS(xTaskGetTickCount())/1000) >= SMS_time){
			current_time=pdTICKS_TO_MS(xTaskGetTickCount())/1000;
			SMS_time += 10;
			SMS_check();
			// printf("Siguiente ciclo en 10 segundos\r\n");
			// printf("SMS CHECK tomo %lu segundos\r\n",(pdTICKS_TO_MS(xTaskGetTickCount())/1000-current_time));
			vTaskDelay(100);
        }
        
        if ((pdTICKS_TO_MS(xTaskGetTickCount())/1000) >= OTA_md_time){
			current_time=pdTICKS_TO_MS(xTaskGetTickCount())/1000;
			OTA_md_time += 60;
			OTA_Modem_Check();
			printf("Siguiente ciclo en 60 segundos\r\n");
			printf("OTA CHECK tomo %lu segundos\r\n",(pdTICKS_TO_MS(xTaskGetTickCount())/1000-current_time));
			vTaskDelay(100);
		}

        if(Modem_check_AT()!=MD_AT_OK){
			WAIT_S(2);
            if(Modem_check_AT()!=MD_AT_OK){
                Modem_turn_OFF();
                // check if modem turn ON
                if(Active_modem()!= MD_CFG_SUCCESS) esp_restart();
                ret_update_time=Modem_update_time(1);
			}
		}
		WAIT_S(1);
	}
	vTaskDelete(NULL);
}



void app_main(void){
    ESP_LOGW(TAG, "--->> INIT PROJECT <<---");
	int ret_main = 0;

    const esp_partition_t *partition = esp_ota_get_running_partition();
    switch (partition->address) {
        case 0x00010000:
            ESP_LOGI(TAG, "Running partition: factory");
            break;
        case 0x00110000:
            ESP_LOGI(TAG, "Running partition: ota_0");
            break;
        case 0x00210000:
            ESP_LOGI(TAG, "Running partition: ota_1");
            break;
        default:
            ESP_LOGE(TAG, "Running partition: unknown");
        break;
    }

    // check if an OTA has been done, if so run diagnostics
    esp_ota_img_states_t ota_state;
    if (esp_ota_get_state_partition(partition, &ota_state) == ESP_OK) {
        if (ota_state == ESP_OTA_IMG_PENDING_VERIFY) {
            ESP_LOGI(TAG, "An OTA update has been detected.");
            if (run_diagnostics()) {
                ESP_LOGI(TAG,"Diagnostics completed successfully! Continuing execution.");
                esp_ota_mark_app_valid_cancel_rollback();
            } else {
                ESP_LOGE(TAG,"Diagnostics failed! Start rollback to the previous version.");
                esp_ota_mark_app_invalid_rollback_and_reboot();
            }
        }
    }

    /*--- Initialize NVS ---*/
    Init_NVS_Keys();
    BLE_init();
    // init ble ota--
    // init_ble_ota();

    // Init Modem params
    main_cfg_parms();
    Modem_config();

    // INIT OTA BLE 
    xTaskCreate(Modem_rx_task, "M95_rx_task", 1024*4, NULL, configMAX_PRIORITIES -1,&UART_task_handle);    // active service

    ret_main = Active_modem();
    if(ret_main != MD_CFG_SUCCESS) esp_restart();

	ESP_LOGW(TAG,"-->> END CONFIG <<--\n");

    ret_update_time=Modem_update_time(3);
    ESP_LOGI(TAG, "RET update time: %d",ret_update_time);
    time(&data_modem.time);
    data_modem.signal = Modem_get_signal();
    strcpy(data_modem.code,PROJECT_VER);

    // char* date=epoch_to_string(data_modem.time);
	ESP_LOGI(TAG,"IMEI: %s", data_modem.info.imei);
    ESP_LOGI(TAG,"ICID: %s", data_modem.info.iccid);
    ESP_LOGI(TAG,"FIRMWARE: %s", data_modem.info.firmware);
    ESP_LOGI(TAG,"UNIX: %lld",data_modem.time);
    ESP_LOGI(TAG,"CODE: %s", data_modem.code);
	ESP_LOGI(TAG,"SIGNAL: %d", data_modem.signal);
    
    doc = cJSON_CreateObject();
    cJSON_AddItemToObject(doc,"imei",cJSON_CreateString(data_modem.info.imei));
    cJSON_AddItemToObject(doc,"project",cJSON_CreateString(PROJECT_NAME));
    cJSON_AddItemToObject(doc,"ota",cJSON_CreateString("true"));
    cJSON_AddItemToObject(doc,"cmd",cJSON_CreateString("false"));
    cJSON_AddItemToObject(doc,"sw",cJSON_CreateString("1.1"));
    cJSON_AddItemToObject(doc,"hw",cJSON_CreateString("1.1"));
    cJSON_AddItemToObject(doc,"otaV",cJSON_CreateString("1.0"));
    output = cJSON_PrintUnformatted(doc);

    ESP_LOGI(TAG,"Mensaje OTA:");
    printf(output);
    printf("\r\n");

    //----------mensaje de iniciacion--------//
	sprintf(buff_aux,"IMEI: %s\n"
					"ip_mqtt: %s\n"
					"project: %s\n"
					"code_ver: %s\n",
					data_modem.info.imei,
					ip_mqtt_connect,
					PROJECT_NAME,
					PROJECT_VER);
	ret_main = Modem_SMS_Send(buff_aux,"+51936910211");
	ESP_LOGI(TAG, "send sms status :0x%X",ret_main);
    WAIT_S(5);

	OTA_md_time     = pdTICKS_TO_MS(xTaskGetTickCount())/1000 + 60;
    MQTT_read_time  = pdTICKS_TO_MS(xTaskGetTickCount())/1000 + 15;
	Info_time       = pdTICKS_TO_MS(xTaskGetTickCount())/1000 + 10;
    SMS_time        = pdTICKS_TO_MS(xTaskGetTickCount())/1000 + 5;
	current_time    = pdTICKS_TO_MS(xTaskGetTickCount())/1000;
    
    AlertQueue = xQueueCreate(5, sizeof(int));
    xTaskCreate(Main_Task,"Main_Task",1024*10,NULL,10, & MAIN_task_handle);
    xTaskCreate(BLE_scann_task,"BLE_task",1024*8,NULL,13, &BLE_Task_handle);
    xTaskCreate(AlarmTask,"AlarmTask",1024*2,NULL, 5, &ALARM_Task_handle);
}

