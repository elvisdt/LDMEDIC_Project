
#include "main.h"
#include "ctype.h"
#include "esp_system.h"


int extraer_numero(const char* cadena) {
    int numero;
    if (sscanf(cadena, "BLE,T,%d", &numero) == 1) {
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
int extraer_mac_y_nombre(char *cadena, char *mac, char* nombre) {
    // "BLE,ADD,<MAC>,<NAME>";
    // Se ignora la parte inicial "BLE,A,"
    if (sscanf(cadena, "BLE,A,%[^,],%[^,]", mac, nombre) == 2) {
        printf("Dirección MAC: %s\n", mac);
        printf("Nombre: %s\n", nombre);
        return 1; // OK
    } 
    return 0; // FAIL
}

// Función para extraer la dirección MAC y los limites de temperatura maxima y minima 
int extraer_mac_tmax_tmin(char *cadena, char *mac, float* tmax, float* tmin) {
    // "BLE,C,<MAC>,<TMAX>,<TMIN>";
    // Se ignora la parte inicial "BLE,C,"
    if (sscanf(cadena,"BLE,C,%[^,],%f,%f", mac,tmax,tmin) == 3) {
        printf("Dirección MAC: %s\n", mac);
        printf("tmax: %.2f, tmin: %.2f\n",*tmax,*tmin);
        return 1; // OK
    } 
    return 0; // FAIL
}
