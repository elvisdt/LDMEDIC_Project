

#include <stdio.h>
#include <string.h>
#include <ctype.h>
#include "esp_log.h"

#include "inkbird_ble.h"

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

//-----------------------------------------------//

void print_ble_record(ink_ble_report_t ble_rep) {
    printf("Name: %s", ble_rep.ble_info.name);

    uint8_t addr[LEN_ADDR_BLE];
    memcpy(addr, ble_rep.ble_info.addr, LEN_ADDR_BLE);

    printf( ", addr: %02X:%02X:%02X:%02X:%02X:%02X", addr[0], addr[1], addr[2], addr[3], addr[4], addr[5]);

    printf(",Data: ");
    for (int i = 0; i < sizeof(ble_rep.ble_data.manuf_data); i++) {
        printf("%02X ", ble_rep.ble_data.manuf_data[i]);
    }
    printf("time: %lld \n", ble_rep.ble_data.time);
}


int ink_addr_to_string(const uint8_t addr[LEN_ADDR_BLE], char* addr_str){

    // Formatear la dirección MAC en una cadena de texto legible
    sprintf(addr_str, "%02X:%02X:%02X:%02X:%02X:%02X",
            addr[0], addr[1], addr[2], addr[3], addr[4], addr[5]);

    // Verificar si la conversión fue exitosa
    if (strlen(addr_str) == 17) {
        return 0; // Éxito
    } else {
        return -1; // Error
    }
}


int ink_addr_to_string_1(const uint8_t addr[LEN_ADDR_BLE], char* addr_str){

    // Formatear la dirección MAC en una cadena de texto legible sin los dos puntos
    sprintf(addr_str, "%02X%02X%02X%02X%02X%02X",
            addr[0], addr[1], addr[2], addr[3], addr[4], addr[5]);

    // Verificar si la conversión fue exitosa
    if (strlen(addr_str) == 12) {
        return 1; // Éxito
    } else {
        return 0; // Error
    }
}

// Función para convertir una cadena de texto que representa una dirección MAC a esp_bd_addr_t
int ink_string_to_addr(const char *addr_str, uint8_t addr[LEN_ADDR_BLE]){
    // Verifica si la cadena de texto tiene la longitud correcta para una dirección MAC
    if (strlen(addr_str) != 17) {
        printf("Error: La cadena de texto no representa una dirección MAC válida.\n");
        return -1; // Retorna un código de error
    }

    char raw_mac[18]; // Espacio para la dirección MAC sin caracteres de formato

    // Elimina los dos puntos de la cadena de texto de la MAC
    int j = 0;
    for (int i = 0; i < strlen(addr_str); i++) {
        if (isxdigit((unsigned char)addr_str[i])) { // Asegúrate de que el carácter sea unsigned char
            raw_mac[j++] = addr_str[i];
        }
    }
    raw_mac[j] = '\0'; // Asegúrate de que la cadena termine correctamente

    // Divide la cadena de texto de la MAC en 6 segmentos de dos caracteres cada uno
    uint8_t addr_bytes[6];
    for (int i = 0; i < 6; i++) {
        sscanf(raw_mac + 2 * i, "%2hhx", &addr_bytes[i]);
    }

    // Copia los bytes de la dirección MAC al objeto
    memcpy(addr, addr_bytes, sizeof(addr_bytes));

    return 0; // Retorna 1 para indicar éxito
}




/*---------------------------*/
void ink_clean_list_ble_info(ink_list_ble_info_t* list_ble_scan){
    list_ble_scan->num_info=0; // Inicializa con 0 elementos
    for (int i = 0; i < MAX_BLE_DEVICES; i++) {
        memset(list_ble_scan->ls_info[i].name, '\0',strlen(list_ble_scan->ls_info[i].name));
        memset(list_ble_scan->ls_info[i].addr, 0, sizeof(list_ble_scan->ls_info[i].addr));
    }
}

void ink_clean_list_ble_rep(ink_list_ble_report_t* list_ble_rep){
    for (int i = 0; i < MAX_BLE_DEVICES; i++) {
        memset(list_ble_rep->ls_ble[i].ble_info.name, '\0',strlen(list_ble_rep->ls_ble[i].ble_info.name));
        memset(list_ble_rep->ls_ble[i].ble_info.addr, 0, sizeof(list_ble_rep->ls_ble[i].ble_info.addr));
        memset(list_ble_rep->ls_ble[i].ble_data.manuf_data, 0, sizeof(list_ble_rep->ls_ble[i].ble_data.manuf_data));
        list_ble_rep->ls_ble[i].ble_data.time=0;
        list_ble_rep->ls_ble[i].ready=0;
    }
}


void ink_concat_list_ble(ink_list_ble_info_t list_ble, char* buffer) {
     printf("INK: Number BLE register: %d\r\n", list_ble.num_info);
     memset(buffer,'\0',strlen(buffer));
     strcat(buffer,"");
    char mac_str[20] ={0};

    for (int i = 0; i < list_ble.num_info; i++) {
        ink_addr_to_string(list_ble.ls_info[i].addr,mac_str);
        strcat(buffer,mac_str);
        strcat(buffer,", ");
        strcat(buffer,list_ble.ls_info[i].name);
        strcat(buffer,"\r\n");
    }
    return;
}

int ink_get_indx_to_list_reg(uint8_t addr[LEN_ADDR_BLE], ink_list_ble_info_t list_ble_addr, uint8_t* idx){
    // for of list num of device
    for (size_t i = 0; i < list_ble_addr.num_info; i++) {
        if(memcmp(addr,list_ble_addr.ls_info[i].addr,LEN_ADDR_BLE) == 0){
            *idx = i;
            return 0;
        }
    }
    return -1; // NOT FOUND ADDR INDEX
}



int ink_get_indx_to_list_report(uint8_t addr[LEN_ADDR_BLE], ink_list_ble_report_t list_ble_rep){
    // verifica si ya existe en la lista de registrado
    for (size_t i = 0; i < MAX_BLE_DEVICES; i++) {
        if(memcmp(addr,list_ble_rep.ls_ble[i].ble_info.addr,LEN_ADDR_BLE) == 0){
            return i;
        }else if (list_ble_rep.ls_ble[i].ready ==0){
            return i;
        }
    }
    return -1; //
}
