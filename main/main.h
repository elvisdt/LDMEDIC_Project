
#ifndef _MAIN_H_
#define _MAIN_H_

#include <stdio.h>
#include <string.h>

#include "EG915/EG915_modem.h"
#include "inkbird_ble.h"

/************************************************
 * DEFINES
*************************************************/
#define ALARM_ACTIVE    0x01
#define ALARM_DEACTIVE  0X00




/************************************************
 * STRUCTURES
*************************************************/

typedef struct modem_gsm{
    EG915_info_t info;  
    char         code[10];
	int          signal;
	time_t       time;
    uint8_t      num_ble;
    float        bat_vol;
}modem_gsm_t;


typedef struct {
    uint8_t addr[LEN_ADDR_BLE];
    time_t  time;
    uint8_t idx;
}data_alarm_t;


typedef struct {
    char phone[15];
    uint8_t status;
}phone_alrm_t;


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
int m_get_delay(const char* cadena);


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
int m_get_params_ble(char *cadena, ink_ble_info_t* ble_info); 

// Función para extraer la dirección MAC y el nombre
int m_get_alert_phone(char *cadena);

int m_get_temp_alert(ink_ble_report_t data);

void m_epoch_to_str(time_t rawtime, char* buffer, size_t len);

#endif /*_MAIN_H_*/