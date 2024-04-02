
#ifndef _MAIN_H_
#define _MAIN_H_

#include <stdio.h>
#include <string.h>

#include "EG915/EG915_modem.h"
#include "inkbird_ble.h"

/************************************************
 * DEFINES
*************************************************/

/************************************************
 * STRUCTURES
*************************************************/

typedef struct modem_gsm{
    EG915_info_t info;  
    char         code[10];
	int          signal;
	time_t       time;
}modem_gsm_t;


typedef struct {
    char mac[20];
    char name[10];
    float tem_max;
    float tem_min;
}cfg_ble_t;



int validarIP(const char* ip);

char* m_get_esp_rest_reason();


/**
 * Divide una cadena para extraer una dirección IP y la valida.
 *
 * Esta función toma una cadena en formato "MQTT,<IP>", extrae la dirección IP y la valida.
 *
 * @param cadena La cadena de entrada que contiene la información en el formato especificado.
 * @param ip Puntero al buffer donde se almacenará la dirección IP extraída.
 *           Se espera que este buffer tenga suficiente espacio para almacenar la dirección IP.
 * @return Devuelve 1 si se extrajo y validó con éxito la dirección IP, -1 si la dirección IP es inválida,
 *         y 0 si la cadena no está en el formato esperado.
 */
int split_and_check_IP(char* cadena, char* ip);


/**
 * Extrar el numero de la trama "BLE,T,<number>"
*/
int extraer_numero(const char* cadena);


/**
 * Extrae la dirección MAC y el nombre de una cadena en formato "BLE,A,<MAC>,<NAME>".
 *
 * Esta función toma una cadena en formato "BLE,A,<MAC>,<NAME>" y extrae la dirección MAC
 * y el nombre, que se esperan estar separados por comas después de "BLE,A,".
 *
 * @param cadena La cadena de entrada que contiene la información en el formato especificado.
 * @param mac Puntero al buffer donde se almacenará la dirección MAC extraída.
 *            Se espera que este buffer tenga al menos espacio para almacenar una dirección MAC válida.
 * @param nombre Puntero al buffer donde se almacenará el nombre extraído.
 *               Se espera que este buffer tenga suficiente espacio para almacenar el nombre.
 * @return Devuelve 1 si se extrajo con éxito la dirección MAC y el nombre, de lo contrario, devuelve 0.
 */
int extraer_mac_y_nombre(char *cadena, char *mac, char* nombre); 


int extraer_mac_tmax_tmin(char *cadena, char *mac, float* tmax, float* tmin);
/************************************************
 * JASON PARSER
*************************************************/

int js_modem_to_str(const modem_gsm_t modem,int num_ble, char* buffer);

int js_str_to_ble(const char *json_string, cfg_ble_t *ble_config);

int js_record_data_ble(ink_sens_ble_t data_ble, char *buffer);

#endif /*_MAIN_H_*/