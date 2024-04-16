

#include <stdio.h>
#include <stdbool.h>
#include <string.h>
#include <stdlib.h>
#include <inttypes.h>
#include "math.h"

#include "cJSON.h"
#include "inkbird_ble.h"
#include "main.h"

#include "jason_parser.h"

//-------------------------------------//
//-------------------------------------//


int js_modem_to_str(const modem_gsm_t modem, char* buffer){
    cJSON *root = cJSON_CreateObject();
    if (root == NULL) {
        printf("Error al crear el objeto JSON\n");
        return -1;
    }
    cJSON_AddStringToObject(root, "iccid",   modem.info.iccid);
    cJSON_AddStringToObject(root, "code",    modem.code);
    cJSON_AddNumberToObject(root, "n-ble",  modem.num_ble);
    cJSON_AddNumberToObject(root, "time",    modem.time);
    cJSON_AddNumberToObject(root, "signal",  modem.signal);
    char *json = cJSON_PrintUnformatted(root);
    sprintf(buffer,"%s\r\n",json);
    cJSON_Delete(root);
    
    free(json);
    json=NULL;
    return 0;
}


/**********************************************************
 * 
**********************************************************/

int js_record_data_ble(ink_ble_report_t data, char *buffer){
    memset(buffer,'\0',strlen(buffer));
    cJSON *root = cJSON_CreateObject();
    if (root == NULL) {
        printf("Error al crear el objeto JSON\n");
        return 0;
    }
    double temp = (double)Inkbird_temperature(data.ble_data.manuf_data);
    double hum  =  (double)Inkbird_humidity(data.ble_data.manuf_data);
    double battery =  (double)Inkbird_battery(data.ble_data.manuf_data);

    battery = round(battery*1e2)/1e2;
    temp = round(temp*1e2)/1e2;
    hum = round(hum*1e2)/1e2;

    cJSON_AddNumberToObject(root, "time", data.ble_data.time);
    cJSON_AddStringToObject(root, "name", data.ble_info.name);
    cJSON_AddNumberToObject(root, "rssi", data.ble_data.rssi);
    cJSON_AddNumberToObject(root, "tem", temp);
    cJSON_AddNumberToObject(root, "hum", hum);
    cJSON_AddNumberToObject(root, "bat", battery);
    

    cJSON *limit = cJSON_CreateObject();
    cJSON_AddNumberToObject(limit, "md", data.ble_info.limits.mode);
    cJSON_AddNumberToObject(limit, "Tmx", data.ble_info.limits.Tmax);
    cJSON_AddNumberToObject(limit, "Tmn",  data.ble_info.limits.Tmin);
    cJSON_AddItemToObject(root, "lmt", limit);

    char *json = cJSON_PrintUnformatted(root);
    sprintf(buffer,"%s\r\n",json);
    cJSON_Delete(root);
    free(json);
    json = NULL;

    return 1;
}


