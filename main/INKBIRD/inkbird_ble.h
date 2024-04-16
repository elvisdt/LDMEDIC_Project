/**
 * @file inkbird_ble.h
 * @brief This file contains the declarations for functions and structures
 *        related to Inkbird Bluetooth Low Energy (BLE) devices.
 */

#ifndef _INKBIRD_BLE_H
#define _INKBIRD_BLE_H

#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <time.h>

/*********************************************************
 * DEFINES
*********************************************************/
#define MAX_BLE_DEVICES 10
#define LEN_ADDR_BLE 6

#define ALERT_BLE_TMAX    1
#define ALERT_BLE_TMIN    2

/*********************************************************
 * STRUCTURES
*********************************************************/
typedef struct{
    int Tmax;
    int Tmin;
    uint8_t mode;   // active or deactive;
}ink_temp_lim_t;


typedef struct{
    char    name[10]; // custom name
    uint8_t addr[LEN_ADDR_BLE];  // addr
    ink_temp_lim_t limits;
}ink_ble_info_t;

typedef struct{
    uint8_t manuf_data[9];  // manuf data
    time_t  time;
    int     rssi;
}ink_ble_data_t;


/*-------STRUCTURA PARA GURADAR LA LISTA DE DISPOSITIVOS------*/
typedef struct {
    uint8_t num_info;
    ink_ble_info_t ls_info[MAX_BLE_DEVICES];
}ink_list_ble_info_t;


/*-------STRUCTURA DE TODAS LAS CARACTERITICAS------*/
typedef struct{
    ink_ble_info_t  ble_info;    // char 
    ink_ble_data_t  ble_data;    // las time update data
    uint8_t ready;              // data is ready
}ink_ble_report_t;



/*-------STRUCTURA DE TODAS LAS CARACTERITICAS PARA REPORTAR------*/
typedef struct {
    ink_ble_report_t ls_ble[MAX_BLE_DEVICES];
}ink_list_ble_report_t;


/**
 * @brief Finds the index of a device by addr address.
 *
 * This function searches for a device with a specific addr address within
 * the given array of devices.
 *
 * @param datos_guardados Array of stored device information.
 * @param addr_addr addr address to search for.
 * @return Index of the device in the array, or -1 if not found.
 */
int Inkbird_addr_index(ink_list_ble_info_t  list_dev, uint8_t addr[6]);




/**
 * @brief Extracts temperature from manufacturer data.
 *
 * This function extracts temperature information from the manufacturer data
 * received from an Inkbird BLE device.
 *
 * @param manufacturer_data Pointer to the manufacturer data.
 * @return Temperature value in degrees Celsius.
 */
float Inkbird_temperature(const uint8_t *manufacturer_data);



/**
 * @brief Extracts humidity from manufacturer data.
 *
 * This function extracts humidity information from the manufacturer data
 * received from an Inkbird BLE device.
 *
 * @param manufacturer_data Pointer to the manufacturer data.
 * @return Humidity value in percentage.
 */
float Inkbird_humidity(const uint8_t *manufacturer_data);



/**
 * @brief Extracts battery level from manufacturer data.
 *
 * This function extracts battery level information from the manufacturer data
 * received from an Inkbird BLE device.
 *
 * @param manufacturer_data Pointer to the manufacturer data.
 * @return Battery level in percentage.
 */
float Inkbird_battery(const uint8_t *manufacturer_data);


/**
 * Convierte la dirección MAC en una cadena de texto legible con dos puntos.
 * 
 * @param addr       Dirección MAC a convertir.
 * @param addr_str   Cadena de texto donde se almacenará la dirección MAC convertida.
 * @return           0 si la conversión fue exitosa, -1 si hubo un error.
 */
int ink_addr_to_string(const uint8_t addr[LEN_ADDR_BLE], char* addr_str);


/**
 * Convierte la dirección MAC en una cadena de texto legible sin dos puntos.
 * 
 * @param addr       Dirección MAC a convertir.
 * @param addr_str   Cadena de texto donde se almacenará la dirección MAC convertida.
 * @return           1 si la conversión fue exitosa, 0 si hubo un error.
 */
int ink_addr_to_string_1(const uint8_t addr[LEN_ADDR_BLE], char* addr_str);


/**
 * Convierte una cadena de texto que representa una dirección MAC a esp_bd_addr_t.
 * 
 * @param addr_str   Cadena de texto que representa la dirección MAC.
 * @param addr       Puntero a esp_bd_addr_t donde se almacenará la dirección MAC convertida.
 * @return           0 si la conversión fue exitosa, -1 si hubo un error.
 */
int ink_string_to_addr(const char *addr_str, uint8_t addr[LEN_ADDR_BLE]);


/**
 * Inicializa la lista de información BLE.
 * 
 * @param list_ble_scan   Puntero a la estructura ink_list_ble_info_t.
 */
void ink_clean_list_ble_info(ink_list_ble_info_t* list_ble_scan);


/**
 * Inicializa la lista de información BLE.
 * 
 * @param list_ble_rep   Puntero a la estructura ink_list_ble_report_t.
 */
void ink_clean_list_ble_rep(ink_list_ble_report_t* list_ble_rep);

/**
 * Concatena la lista de información BLE en un búfer.
 * 
 * @param list_ble   Estructura ink_list_ble_info_t con la información BLE.
 * @param buffer     Búfer donde se almacenará la lista concatenada.
 */
void ink_concat_list_ble(ink_list_ble_info_t list_ble, char* buffer);


/**
 * Obtiene el índice en la lista de registros BLE.
 * 
 * @param addr              Dirección MAC para buscar en la lista.
 * @param list_ble_addr     Estructura ink_list_ble_info_t con la lista de registros BLE.
 * @param idx               uint8_t con el indice encontrado de la lista.
 * @return                  El índice si se encuentra la dirección MAC, -1 si no se encuentra.
 */

int ink_get_indx_to_list_reg(uint8_t addr[LEN_ADDR_BLE], ink_list_ble_info_t list_ble_addr, uint8_t* idx);

int ink_get_indx_name_to_list_reg(char* name, ink_list_ble_info_t list_ble_addr, uint8_t* idx);

/**
 * Obtiene el índice en la lista de informes BLE.
 * 
 * @param addr              Dirección MAC para buscar en la lista.
 * @param list_ble_rep      Estructura ink_list_ble_report_t con la lista de informes BLE.
 * @return                  El índice si se encuentra la dirección MAC, el número de dispositivos si no se encuentra.
 */
int ink_get_indx_to_list_report(uint8_t addr[LEN_ADDR_BLE], ink_list_ble_report_t list_ble_rep);



#endif // _INKBIRD_BLE_H
