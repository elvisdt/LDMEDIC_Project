#include <stdio.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <inttypes.h>

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>
#include <freertos/event_groups.h>
#include <esp_event.h>

#include <sdkconfig.h>
#include <esp_system.h>
#include <sys/time.h>
#include <time.h>
#include <esp_timer.h>
#include <esp_log.h>
#include "esp_ota_ops.h"
#include <nvs.h>
#include <nvs_flash.h>

/*------ble devices----*/
#include "esp_nimble_hci.h"
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "host/ble_hs.h"
#include "host/ble_gap.h"
#include "services/gap/ble_svc_gap.h"


#include <cJSON.h>

/* main libraries */
#include "credentials.h"
#include "EG915_modem.h"
#include "ota_modem.h"
#include "main.h"
#include "inkbird_ble.h"
#include "jason_parser.h"


#include "modem_aux.h"

/* ota modem librarias */
#include "crc.h"
#include "ota_control.h"
#include "ota_esp32.h"
#include "ota_headers.h"

/* ota ble librarias */
// #include "gap.h"
// #include "gatt_svr.h"


/***********************************************
 * DEFINES
************************************************/

#define TAG             "MAIN"
#define MAX_ATTEMPS     3


#define MASTER_TOPIC_MQTT   "LDMEDIC"

#define NAMESPACE_NVS		"storage"
#define KEY_BLE_LIST		"BLE_LIST"
#define KEY_IP_MQTT			"IP_MQTT"
#define KEY_BLE_TIME		"BLET"


/*----> >-----*/
#define     PIN_CIRCULINA      GPIO_NUM_11
#define	    INIT_DELAY_BLE		3

#define     WAIT_MS(x)		vTaskDelay(pdMS_TO_TICKS(x))
#define     WAIT_S(x)		vTaskDelay(pdMS_TO_TICKS(x*1e3))

/***********************************************
 * STRUCTURES
************************************************/

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


/*--> NVS <---*/
nvs_handle_t storage_nvs_handle;
uint16_t delay_ble=INIT_DELAY_BLE;
uint8_t     delay_tmax = 20;
uint8_t     delay_tmin = 20;

size_t BLE_size  = sizeof(ink_list_ble_info_t);
size_t MQTT_size = 50;



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


/*--> gloabl main varibles<--*/
ink_list_ble_info_t   list_ble_info={0};
ink_list_ble_report_t list_ble_report={0};

uint32_t    Info_time=0;
uint32_t    MQTT_read_time=0;
uint32_t    OTA_md_time=1;
uint32_t    SMS_time=0;
uint32_t    BLE_time=0;

//size_t BLE_size  = sizeof(ink_list_ble_addr_t);
char ip_mqtt_connect[50] = ip_MQTT;


/*---> ble parameters <---*/
uint8_t ble_addr_type;
uint8_t active_scan_process = 0;   // Variable global para habilitar si procesar los eventos de escaneo o no
int ret_init_scan=0;

/***********************************************
 * FUNCIONTIONS
************************************************/
void ble_app_scan(void);


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
	err = nvs_get_blob(storage_nvs_handle, KEY_BLE_LIST, &list_ble_info, &BLE_size);
	if(err == ESP_ERR_NVS_NOT_FOUND){
		ink_clean_list_ble_info(&list_ble_info);
		nvs_set_blob(storage_nvs_handle, KEY_BLE_LIST, &list_ble_info, sizeof(ink_list_ble_info_t));     
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
	
	ink_concat_list_ble(list_ble_info,buff_aux);
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
/****************************************************
 * BLE PROCESS
*****************************************************/
// BLE event handling
static int ble_gap_event(struct ble_gap_event *event, void *arg){

    if (active_scan_process==0) {
        // ESP_LOGW("GAP", "ble suspend");
        return 0;
    }
    
    struct ble_hs_adv_fields fields;
    uint8_t addr_scan[LEN_ADDR_BLE]={0};

    switch (event->type){
    // NimBLE event discovery
    case BLE_GAP_EVENT_DISC:
        
        // ESP_LOGW("GAP", "GAP EVENT DISCOVERY");
        ble_hs_adv_parse_fields(&fields, event->disc.data, event->disc.length_data);
        // procesar informacion de dispositivos escaneandos basjo estas condiciones
        if ((fields.name_len>0) && (list_ble_info.num_info>0) && (fields.mfg_data_len==9)){  
            WAIT_MS(200);          
			for (size_t i = 0; i < LEN_ADDR_BLE; i++) {
				addr_scan[i] = event->disc.addr.val[LEN_ADDR_BLE - 1 - i];
			}
			// verifcamos si la mac existe en nuestra lista y obtnemos el indice
			int idx_mac= ink_get_indx_to_list_reg(addr_scan, list_ble_info);
			char mac_str_aux[20];
			ink_addr_to_string(addr_scan, mac_str_aux);
			
			// validamos el indice retornado;
			if (idx_mac>=0){
                ESP_LOGI("GAP","name: %.*s", fields.name_len, fields.name);
                ESP_LOGI("GAP","addr: %s, idx %d\r\n", mac_str_aux, idx_mac);
                WAIT_MS(500);
				// obtenemos el indice para registrar la data
				int idx_rep= ink_get_indx_to_list_report(addr_scan, list_ble_report);
				if (idx_rep>=0){
					// update chars
					list_ble_report.ls_ble[idx_rep].ble_info = list_ble_info.ls_info[idx_mac];
					// ipdate manuf data
					memcpy(list_ble_report.ls_ble[idx_rep].ble_data.manuf_data,fields.mfg_data,fields.mfg_data_len);
					// update time
					time(&list_ble_report.ls_ble[idx_rep].ble_data.time);
					// update state
					list_ble_report.ls_ble[idx_rep].ready = 1;
				}
			}else{
                ESP_LOGW("GAP","name: %.*s", fields.name_len, fields.name);
                ESP_LOGW("GAP","addr: %s \r\n", mac_str_aux);
            }
        }
        break;

    default:
        break;
    }
    return 0;
}

void ble_app_scan(void){
    ESP_LOGW("BLE","Init NimBLE scann...");
    struct ble_gap_disc_params disc_params;
    disc_params.filter_duplicates = 0;
    disc_params.passive = 0;
    disc_params.itvl    = 0xF000;  	// 0x0004 -> 0xFFFF (0.625 ms)
    disc_params.window  = 0x0400; 	// 0x0004 -> 0xFFFF (0.625 ms)
    disc_params.filter_policy = BLE_HCI_SCAN_FILT_NO_WL;
    disc_params.limited = 0; // BLE_HS_FOREVER; // Escaneo continuo
    ret_init_scan = ble_gap_disc(ble_addr_type, BLE_HS_FOREVER, &disc_params, ble_gap_event, NULL);
	ESP_LOGW("BLE","ret scan: %d\r\n", ret_init_scan);
}

// The application
void ble_app_on_sync(void){
    int ret;
	ret = ble_hs_id_infer_auto(0, &ble_addr_type); // Determines the best address type automatically
    ESP_LOGI("BLE","ret ble MAC: %d",ret);
	ble_app_scan();                          
}

// The infinite task
void host_task(void *param){
    nimble_port_run(); // This function will return only when nimble_port_stop() is executed
}


void init_ble_scann(){
    /*------------*/
    char str_imei[30]={0};
    sprintf(str_imei,"gtw-%s",data_modem.info.imei);
    // nvs_flash_init();                            // 1 - Initialize NVS flash using
    nimble_port_init();                             // 2 - Initialize the controller stack
    ble_svc_gap_device_name_set(str_imei);          // 3 - Set device name characteristic
    ble_svc_gap_init();                             // 4 - Initialize GAP service
    ble_hs_cfg.sync_cb = ble_app_on_sync;           // 5 - Set application
    nimble_port_freertos_init(host_task);           // 6 - Set infinite task
    active_scan_process = 1;
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

            else{
                int ret_tcp=TCP_close(); // close tcp
                printf("ret : 0x%x\r\n",ret_tcp);
            }

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
	
    data_modem.num_ble = list_ble_info.num_info;

	js_modem_to_str(data_modem, buff_aux);
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

		for (size_t i = 0; i < MAX_BLE_DEVICES; i++){
            if (list_ble_report.ls_ble[i].ready==0){
                break;
            }

            ink_addr_to_string(list_ble_report.ls_ble[i].ble_info.addr, mac_aux);
			sprintf(topico,"%s/%s/%s",MASTER_TOPIC_MQTT,data_modem.info.imei,mac_aux);

			js_record_data_ble(list_ble_report.ls_ble[i], buff_aux);
			data_len = strlen(buff_aux);
			Modem_Mqtt_Pub(buff_aux,topico,data_len,mqtt_idx, 0);
			WAIT_S(1);
		}
        
	}
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
                    active_scan_process = 0; // DEACTIVE SCAN PROCCES
                    WAIT_S(1);

                    uint8_t addr_aux[LEN_ADDR_BLE];
                    ret_sms=ink_string_to_addr(mac_str,addr_aux);
                    ESP_LOGI(TAG, "SMS mac err: %d",ret_sms);
                    if (ret_sms==0){
                        int idx_info = ink_get_indx_to_list_reg(addr_aux, list_ble_info);
                        if ( idx_info==-1 && list_ble_info.num_info<MAX_BLE_DEVICES){
                            memcpy(list_ble_info.ls_info[list_ble_info.num_info].addr,addr_aux, LEN_ADDR_BLE);
                            strcpy(list_ble_info.ls_info[list_ble_info.num_info].name,nombre);
                            list_ble_info.num_info +=1;
                            ESP_LOGI(TAG_SMS,"BLE ADD OK");
                        }else{
                            ESP_LOGW(TAG_SMS,"BLE ADD FAIL- DEVICE EXIST");
                        }
                    }else{
                        ESP_LOGE(TAG_SMS,"BLE ADD FAIL");
                    }

                    active_scan_process = 1; // ACTIVE SCAN PROCCES
                }else{
                    ESP_LOGE(TAG_SMS,"MAC IS INCORRECT");
                }
                
			}else if(strstr(message,"BLE,C,")!=NULL){
                char mac_str[18];   // MAC (formato XX:XX:XX:XX:XX)
                float tmax;
                float tmin;
                ret_sms=extraer_mac_tmax_tmin(message,mac_str,&tmax,&tmin);
                ESP_LOGI(TAG, "SMS extrac mac name: %d",ret_sms);
                /*
                if (ret_sms==1)	{
                    esp_bd_addr_t addr_aux;
                    // se convierte el mac en ADDR
                    ret_sms= ink_string__to__esp_bd_addr(mac_str,&addr_aux);
                    ESP_LOGI(TAG, "SMS mac err: %d",ret_sms);
                    if (ret_sms==1){

                        int idx_list =-1;
                        ret_sms=ink_check_indx_ble(addr_aux,list_ble_device,&idx_list);
                        ESP_LOGI(TAG, "exist err: %d",ret_sms);
                        if (ret_sms==1){
                            ink_cfg_tem_t cfg_tem_aux={1,tmax,tmin};
                            list_ble_device.addr_scan[idx_list].cfg_tem = cfg_tem_aux;
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
                */
			}else if(strstr(message,"BLE,E")!=NULL){
                active_scan_process = 1; // activar
                WAIT_S(1);
				ink_clean_list_ble_info(&list_ble_info);
				ink_clean_list_ble_rep(&list_ble_report);
				ESP_LOGW(TAG_SMS,"lista BLE eliminada");
                active_scan_process = 1; // activar

			}
		}else if(strstr(message,"INFO")!=NULL){
			enviar_sms=1;
			ESP_LOGI(TAG_SMS,"Informacion solicitada");
		}else if(strstr(message,"RESET")!=NULL){
			ESP_LOGE(TAG_SMS,"Reset solicitado");
			Modem_turn_OFF();
            WAIT_S(1);
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

		nvs_set_u16(storage_nvs_handle, KEY_BLE_TIME,delay_ble);
		nvs_set_blob(storage_nvs_handle, KEY_BLE_LIST, &list_ble_info, sizeof(ink_list_ble_info_t));
		nvs_set_str(storage_nvs_handle, KEY_IP_MQTT,ip_mqtt_connect);

		nvs_commit(storage_nvs_handle); // guardar cambios de maenra permanente
		/*===================================================*/
        ret_sms =Modem_SMS_delete();
        ESP_LOGI(TAG_SMS, "delete sms :0x%X",ret_sms);
	}

	if (enviar_sms){
		enviar_sms=0;
		memset(message, '\0',strlen(message));
		ink_concat_list_ble(list_ble_info,buff_aux);

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

        ret_sms =Modem_SMS_delete();
        ESP_LOGI(TAG_SMS, "delete sms :0x%X",ret_sms);
	}
    return;
}


/*----------------------------------------------------
 * WTD TASK
*----------------------------------------------------*/
static void M95_Watchdog(void* pvParameters){
	uint32_t watchdog_time;
	for(;;){
		watchdog_time=((pdTICKS_TO_MS(xTaskGetTickCount())/1000)-current_time);
		if (watchdog_time >=180 && watchdog_en){
			ESP_LOGE("WTD"," moden no respondio por 3 minutos, reiniciando...\r\n");
			vTaskDelete(MAIN_task_handle);
            Modem_turn_OFF();
            WAIT_S(2);
			esp_restart();
		}

		if (watchdog_en){
			printf("WTD. time pass: %lu sec\r\n",watchdog_time);
		}
		vTaskDelay(5000/portTICK_PERIOD_MS);
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
			Info_time+= 5*60;// cada 5 min
			if(ret_update_time!=MD_CFG_SUCCESS){
			    ret_update_time=Modem_update_time(1);
			}
            Info_Send();
            WAIT_S(1);
		}

		if ((pdTICKS_TO_MS(xTaskGetTickCount())/1000) >= BLE_time){
			current_time=pdTICKS_TO_MS(xTaskGetTickCount())/1000;
			BLE_time += delay_ble*60;
			printf("----send ble time---\r\n");

			active_scan_process=0; // no proceso
			WAIT_S(1);
			if (list_ble_info.num_info>0){
				BLE_send();
			}
			active_scan_process = 1;
			printf("BLE SEND in %u mins\r\n",delay_ble);
			current_time=pdTICKS_TO_MS(xTaskGetTickCount())/1000;
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
			SMS_time += 5;
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

    // Init Modem params
    main_cfg_parms();
    Modem_config();
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
					"code_ver: %s",
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
    xTaskCreate(AlarmTask,"AlarmTask",1024*2,NULL, 5, &ALARM_Task_handle);
    xTaskCreate(M95_Watchdog,"M95_Watchdog",2048, NULL,11,NULL);

    init_ble_scann();
    WAIT_S(2);
    if (ret_init_scan!=0){
		vTaskDelete(MAIN_task_handle);
		esp_restart();
	}
}

