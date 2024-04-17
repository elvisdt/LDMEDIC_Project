
#include "main.h"
#include "ctype.h"
#include "esp_system.h"


int m_get_delay(const char* cadena) {
    int numero;

    if (sscanf(cadena, "%*[^,],T,%d", &numero) == 1) {
        if (numero >= 0 && numero <= 99) {
            return numero;
        } else {
            return -1;
        }
    } else {
        return -1;
    }
}



int validarIP(const char* ip) {
    char ip_copy[50]; 
    strcpy(ip_copy, ip); // Copia la dirección IP 

    int numTokens = 0;
    char* token = strtok(ip_copy, ".");

    while (token != NULL) {
        // Elimina espacios vacíos y caracteres no numéricos
        char cleaned_token[4];
        int i = 0;
        for (int j = 0; token[j] != '\0'; j++) {
            if (token[j] >= '0' && token[j] <= '9') {
                cleaned_token[i++] = token[j];
            }else{
                return -2; // CARACTERES INVALIDOS
            }
        }
        cleaned_token[i] = '\0';

        // Verifica si el token es un número válido
        char* endptr;
        int num = strtol(cleaned_token, &endptr, 10);
        if (*endptr != '\0' || num < 0 || num > 255) {
            return -1;
        }
        numTokens++;
        token = strtok(NULL, ".");
    }
    if (numTokens == 4){
        return 1;   // validate succesfull
    }
    return 0;   // fail num tokens
}


int split_and_check_IP(char* cadena, char* ip) {
    int ret = 0;
    if (sscanf(cadena, "MQTT,%[^,]", ip) == 1) {
        printf("Dirección MAC: %s\n", ip);
        ret =validarIP(ip);
        if (ret==1){
            return 1;// validate succesfull
        }else{
            return -1; // fail validate
        }
    } 
    return 0; // FAIL
}




// Función para extraer la dirección MAC y el nombre
int m_get_params_ble(char *cadena, ink_ble_info_t* ble_info){
    // "BLE,ADD,<MAC>,<NAME>,[<TMAX>,<TMIN>]";
    // BLE,A,49:23:04:08:24:62,N1F,-20.2,15.5
    int ret=0;

    char addr_aux[20]={0};
    char name_aux[10]={0};
    int t_n1,t_n2;

    int n_scan = sscanf(cadena, "%*[^,],%*[^,],%[^,],%[^,],%d,%d", addr_aux, name_aux, &t_n1,&t_n2);

    if (n_scan==2){
        ret=ink_string_to_addr(addr_aux,ble_info->addr);
        if (ret!=0) return -1;
        strcpy(ble_info->name,name_aux);
        ble_info->limits.mode = 0;
        return 0; // SUCCESFULL PARSE

    }else if (n_scan==4){
        ret=ink_string_to_addr(addr_aux,ble_info->addr);
        if (ret!=0) return -1;
        strcpy(ble_info->name,name_aux);
        ble_info->limits.mode = 1;
        ble_info->limits.Tmax = (t_n1 > t_n2) ? t_n1 : t_n2;
        ble_info->limits.Tmin = (t_n1 < t_n2) ? t_n1 : t_n2;
        return 0; // SUCCESFULL PARSE
    }
    
    return -1;
}


// Función para extraer la dirección MAC y el nombre
int m_get_alert_phone(char *cadena){
    // "BLE,ADD,<MAC>,<NAME>,[<TMAX>,<TMIN>]";
    // BLE,A,49:23:04:08:24:62,N1F,-20.2,15.5
    char prefix[4], number[11];
    // Extraer la información después de la coma
    sscanf(cadena, "%*[^,],%s", number);

    // Extraer el prefijo y el número
    sscanf(number, "%3s%10s", prefix, number);

    // Validar que el prefijo es +51 y que el número tiene 9 dígitos
    if (strcmp(prefix, "+51") == 0 && strlen(number) == 9) {
        sprintf(cadena,"%s%s",prefix,number);
        return 0;
    }
    return -1;
}


/**
 * @brief Determina si hay una alerta de temperatura activa o no.
 *
 * Esta función determina si hay una alerta de temperatura activa o no 
 * basada en los datos del informe BLE proporcionados.
 *
 * @param data Informe BLE que contiene la información de temperatura y límites de alarma.
 * @return Retorna ALARM_ACTIVE si la temperatura está fuera del rango de alarma, ALARM_DEACTIVE si está dentro del rango de alarma, o un valor de retorno negativo en caso de error.
 */
int m_get_temp_alert(ink_ble_report_t data){
    // Check alarm status
    float t_min= (float)data.ble_info.limits.Tmin - 0.3;
    float t_max= (float)data.ble_info.limits.Tmax + 0.3;

    if (data.ble_info.limits.mode ==0) {
        return ALARM_DEACTIVE;
    }

    // validate to alarm range 
    float temp= Inkbird_temperature(data.ble_data.manuf_data);
    if ((temp >= t_min) && (temp <= t_max)) {
        return ALARM_DEACTIVE;
    }

    return ALARM_ACTIVE;
}



void m_epoch_to_str(time_t rawtime, char* buffer, size_t len) {
    struct tm * timeinfo;
    rawtime -=5*60; // -5 horas hora local
    timeinfo = localtime(&rawtime);
    strftime(buffer, len, "%Y-%m-%d %H:%M:%S", timeinfo);
}