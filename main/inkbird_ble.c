
#include "inkbird_ble.h"
#include <stdio.h>
#include <string.h>
#include <ctype.h>
#include "esp_log.h"

ink_sens_ble_t * disp_ble;


float Inkbird_temperature(const uint8_t * manufacturer_data){
    int16_t temperatura_raw = manufacturer_data[1] * 256 + manufacturer_data[0];
    float temperature = (float)temperatura_raw;
    temperature = temperature/100;
    return temperature;
}


float Inkbird_humidity(const uint8_t * manufacturer_data){
    int16_t humidity_raw_value = (manufacturer_data[3] * 256 + manufacturer_data[2]);
    float humidity = (float)humidity_raw_value;
    humidity = humidity/100;
    return humidity;
}

float Inkbird_battery(const uint8_t * manufacturer_data){
    float battery = manufacturer_data[7];
    return battery;
}


bool Inkbird_check_empty_mac(esp_bd_addr_t mac_addr){
    uint8_t empty[ESP_BD_ADDR_LEN];
    memset(empty,0x00,ESP_BD_ADDR_LEN);
    if(memcmp(mac_addr,empty,ESP_BD_ADDR_LEN) == 0){
        return true;
    }
    return false;
}


//-----------------------------------------------//

void print_sensor_data(ink_sens_ble_t* devices) {
    printf("Nombre: %s\n", devices->name);

    printf("Dirección: ");
    for (int i = 0; i < sizeof(devices->addr); i++) {
        printf("%02x:", devices->addr[i]);
    }
    printf("\n");

    printf("Datos Hex: ");
    for (int i = 0; i < sizeof(devices->data_Hx); i++) {
        printf("%02x ", devices->data_Hx[i]);
    }
    printf("\n");

    printf("Marca de tiempo: %lld\n", devices->epoch_ts);
}




/********************************************
 * 
********************************************/
// Función para convertir esp_bd_addr_t a cadena de texto y manejar errores
int ink_esp_bd_addr__to__string(esp_bd_addr_t addr, char* str) {
    if (addr == NULL || str == NULL) {
        // Manejo de error: los punteros son nulos
        return 0;
    }

    // Formatear la dirección MAC en una cadena de texto legible
    sprintf(str, "%02X:%02X:%02X:%02X:%02X:%02X",
            addr[0], addr[1], addr[2], addr[3], addr[4], addr[5]);

    // Verificar si la conversión fue exitosa
    if (strlen(str) == 17) {
        return 1; // Éxito
    } else {
        return 0; // Error
    }
}


int ink_esp_bd_addr__to__string_1(esp_bd_addr_t addr, char* str) {
    if (addr == NULL || str == NULL) {
        // Manejo de error: los punteros son nulos
        return 0;
    }

    // Formatear la dirección MAC en una cadena de texto legible sin los dos puntos
    sprintf(str, "%02X%02X%02X%02X%02X%02X",
            addr[0], addr[1], addr[2], addr[3], addr[4], addr[5]);

    // Verificar si la conversión fue exitosa
    if (strlen(str) == 12) {
        return 1; // Éxito
    } else {
        return 0; // Error
    }
}

// Función para convertir una cadena de texto que representa una dirección MAC a esp_bd_addr_t
int ink_string__to__esp_bd_addr(const char *mac_str, esp_bd_addr_t *mac_addr) {
    // Verifica si la cadena de texto tiene la longitud correcta para una dirección MAC
    if (strlen(mac_str) != 17) {
        printf("Error: La cadena de texto no representa una dirección MAC válida.\n");
        return 0; // Retorna un código de error
    }

    char mac_sin_formato[18]; // Espacio para la dirección MAC sin caracteres de formato

    // Elimina los dos puntos de la cadena de texto de la MAC
    int j = 0;
    for (int i = 0; i < strlen(mac_str); i++) {
        if (isxdigit((unsigned char)mac_str[i])) { // Asegúrate de que el carácter sea unsigned char
            mac_sin_formato[j++] = mac_str[i];
        }
    }
    mac_sin_formato[j] = '\0'; // Asegúrate de que la cadena termine correctamente

    // Divide la cadena de texto de la MAC en 6 segmentos de dos caracteres cada uno
    uint8_t addr_bytes[6];
    for (int i = 0; i < 6; i++) {
        sscanf(mac_sin_formato + 2 * i, "%2hhx", &addr_bytes[i]);
    }

    // Copia los bytes de la dirección MAC al objeto esp_bd_addr_t
    memcpy(*mac_addr, addr_bytes, sizeof(esp_bd_addr_t));

    return 1; // Retorna 0 para indicar éxito
}


int ink_check_addr_scan(esp_bd_addr_t addr1, ink_list_ble_addr_t* list_ble_addr, int *idx) {
    char addr_str1[20]={0};
    char addr_dev[20]={0};
    ink_esp_bd_addr__to__string(addr1,addr_str1);
    bool scan_ok_aux = false;
    for (size_t i = 0; i < list_ble_addr->num_ble; i++) {
        scan_ok_aux = list_ble_addr->addr_scan[i].scan_ok;
        if (scan_ok_aux==false){
            ink_esp_bd_addr__to__string(list_ble_addr->addr_scan[i].addr,addr_dev);
            if ( strcmp(addr_str1, addr_dev)==0){
                list_ble_addr->addr_scan[i].scan_ok = true;
                *idx = i;
                return 1; // FOUND ADDR SCAN
            }
        }
    }
    return 0; // NOT FOUND ADDR SCAN
}


/*---------------------------*/
void ink_init_list_ble_addr(ink_list_ble_addr_t* list_ble_scan) {
    list_ble_scan->num_ble = 0; // Inicializa con 0 elementos

    for (int i = 0; i < MAX_BLE_DEVICES; i++) {
        memset(list_ble_scan->addr_scan[i].name, '\0',strlen(list_ble_scan->addr_scan[i].name));
        memset(list_ble_scan->addr_scan[i].addr, 0, sizeof(esp_bd_addr_t));
        list_ble_scan->addr_scan[i].scan_ok = false;
    }
}

void ink_print_list_ble_addr(ink_list_ble_addr_t list_ble) {
    ESP_LOGI("BLE","Number addr of BLE: %d", list_ble.num_ble);
    for (int i = 0; i < list_ble.num_ble; i++) {
        printf("addr %d: \"", i + 1);
        for (int j = 0; j < 6; j++) {
            printf("%02X", list_ble.addr_scan[i].addr[j]);
            if (j < 5) {
                printf(":");
            }
        }
        printf("\" name: \"%s\"\r\n",list_ble.addr_scan[i].name);
    }
}


void ink_concat_lit_ble(ink_list_ble_addr_t list_ble, char* buffer) {
     ESP_LOGI("BLE","Number addr of BLE: %d\r\n", list_ble.num_ble);
     memset(buffer,'\0',strlen(buffer));
     strcat(buffer,"");
    char mac_str[20] ={0};

    for (int i = 0; i < list_ble.num_ble; i++) {
        ink_esp_bd_addr__to__string(list_ble.addr_scan[i].addr,mac_str);
        strcat(buffer,mac_str);
        strcat(buffer,", ");
        strcat(buffer,list_ble.addr_scan[i].name);
        strcat(buffer,"\r\n");
    }
    return;
}

int ink_check_exist_ble(esp_bd_addr_t addr1, ink_list_ble_addr_t list_ble_addr){
    char addr_str1[20]={0};
    char addr_dev[20]={0};
    ink_esp_bd_addr__to__string(addr1,addr_str1);
    for (size_t i = 0; i < list_ble_addr.num_ble; i++) {
        ink_esp_bd_addr__to__string(list_ble_addr.addr_scan[i].addr,addr_dev);
        if ( strcmp(addr_str1, addr_dev)==0){
            return 1; // FOUND ADDR SCAN
        }
    }
    return 0; // NOT FOUND ADDR SCAN
}



int ink_check_limit_temp(ink_sens_ble_t ble_data) {
    // Verificar si la configuración de temperatura está activada
    if (ble_data.cfg_tem.ok == 1) {
        // Calcular la temperatura actual
        float temp = Inkbird_temperature(ble_data.data_Hx);
        // Validar límites de temperatura
        if (temp < ble_data.cfg_tem.tmax && temp > ble_data.cfg_tem.tmin) {
            return ALERT_BLE_DEACTIVE; // La temperatura está dentro de los límites configurados
        } else if (temp > ble_data.cfg_tem.tmax) {
            return ALERT_BLE_TMAX; // La temperatura medida supera el límite máximo configurado
        } else if (temp < ble_data.cfg_tem.tmin) {
            return ALERT_BLE_TMIN; // La temperatura medida está por debajo del límite mínimo configurado
        }
    }
    return ALERT_BLE_DEACTIVE; // La configuración de temperatura no está activada o los límites no están configurados correctamente
}