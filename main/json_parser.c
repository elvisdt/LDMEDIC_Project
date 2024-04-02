
#include <stdio.h>
#include <stdbool.h>
#include <string.h>
#include <stdlib.h>

#include "cJSON.h"
#include "main.h"

//-------------------------------------//


int js_modem_to_str(const modem_gsm_t modem,int num_ble, char* buffer){
    cJSON *root = cJSON_CreateObject();
    if (root == NULL) {
        printf("Error al crear el objeto JSON\n");
        return -1;
    }
    cJSON_AddStringToObject(root, "iccid",   modem.info.iccid);
    cJSON_AddStringToObject(root, "code",    modem.code);
    cJSON_AddNumberToObject(root, "signal",  modem.signal);
    cJSON_AddNumberToObject(root, "time",    modem.time);
    cJSON_AddNumberToObject(root, "ble-num",  num_ble);
    char *json = cJSON_PrintUnformatted(root);
    sprintf(buffer,"%s\r\n",json);
    cJSON_Delete(root);
    
    free(json);
    json=NULL;

    return 0;
}



int js_str_to_ble(const char *json_string, cfg_ble_t *ble_config) {
    cJSON *root = cJSON_Parse(json_string);
    if (root == NULL) {
        const char *error_ptr = cJSON_GetErrorPtr();
        if (error_ptr != NULL) {
            printf("Error parsing JSON: %s\n", error_ptr);
        }
        return -1;
    }

    cJSON *ble_item = cJSON_GetObjectItem(root, "ble");
    if (ble_item != NULL) {
        cJSON *mac_item = cJSON_GetObjectItem(ble_item, "mac");
        cJSON *name_item = cJSON_GetObjectItem(ble_item, "name");
        cJSON *tmax_item = cJSON_GetObjectItem(ble_item, "Tmax");
        cJSON *tmin_item = cJSON_GetObjectItem(ble_item, "Tmin");
        
        if (mac_item && name_item && tmax_item && tmin_item) {
            strncpy(ble_config->mac, mac_item->valuestring, sizeof(ble_config->mac));
            strncpy(ble_config->name, name_item->valuestring, sizeof(ble_config->name));
            ble_config->tem_max = (float)tmax_item->valuedouble;
            ble_config->tem_min = (float)tmin_item->valuedouble;
            cJSON_Delete(root);
            return 0; // Se encontraron todos los campos
        }
    }

    cJSON_Delete(root);
    return -1; // No se encontraron todos los campos
}



int js_record_data_ble(ink_sens_ble_t data_ble, char *buffer){
    memset(buffer,'\0',strlen(buffer));
    cJSON *root = cJSON_CreateObject();
    if (root == NULL) {
        printf("Error al crear el objeto JSON\n");
        return 0;
    }
    char mac_str[20]={0};

    ink_esp_bd_addr__to__string(data_ble.addr,mac_str);
    double temp = (double)Inkbird_temperature(data_ble.data_Hx);
    double hum  =  (double)Inkbird_humidity(data_ble.data_Hx);
    double battery =  (double)Inkbird_battery(data_ble.data_Hx);

    battery = round(battery*1e2)/1e2;
    temp = round(temp*1e2)/1e2;
    hum = round(hum*1e2)/1e2;

    cJSON_AddNumberToObject(root, "time", data_ble.epoch_ts);
    cJSON_AddStringToObject(root, "name", data_ble.name);
    cJSON_AddNumberToObject(root, "tem", temp);
    cJSON_AddNumberToObject(root, "hum", hum);
    cJSON_AddNumberToObject(root, "bat", battery);
    
    if (data_ble.cfg_tem.ok ==1){
        cJSON_AddNumberToObject(root, "tmax", (double)(round(data_ble.cfg_tem.tmax*1e2)/1e2));
        cJSON_AddNumberToObject(root, "tmin", (double)(round(data_ble.cfg_tem.tmin*1e2)/1e2));
    }else{
        cJSON_AddStringToObject(root, "tmax", "Na");
        cJSON_AddStringToObject(root, "tmin", "Na");
    }

    char *json = cJSON_PrintUnformatted(root);
    sprintf(buffer,"%s\r\n",json);
    cJSON_Delete(root);
    free(json);
    return 1;
}