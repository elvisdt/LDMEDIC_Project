/**
 * @file inkbird_ble.h
 * @brief This file contains the declarations for functions and structures
 *        related to Inkbird Bluetooth Low Energy (BLE) devices.
 */

#ifndef INKBIRD_BLE_H
#define INKBIRD_BLE_H

#include <stdint.h>
#include <esp_bt_defs.h>
#include <stdio.h>
#include <string.h>
#include "time.h"
/*******************************************************
 * DEFINES
********************************************************/
#define MAX_BLE_DEVICES 10




#define ALERT_BLE_DEACTIVE      0x00
#define ALERT_BLE_TMAX          0x10
#define ALERT_BLE_TMIN          0x11


/*******************************************************
 * STRUCTURES
********************************************************/
typedef struct {
    uint8_t  ok;
    float tmax;
    float tmin;
}ink_cfg_tem_t;


typedef struct{
    esp_bd_addr_t addr;
    char  name[10];
    bool  scan_ok;
    ink_cfg_tem_t cfg_tem;
}ink_ble_addr_t;

typedef struct {
    int num_ble;
    ink_ble_addr_t addr_scan[MAX_BLE_DEVICES];
}ink_list_ble_addr_t;


typedef struct{
    char   name[10];
    esp_bd_addr_t addr;
    uint8_t data_Hx[9];
    time_t  epoch_ts;
    ink_cfg_tem_t cfg_tem;
}ink_sens_ble_t;


typedef struct {
    bool data_ready;
    int num_ble;
    ink_sens_ble_t data_ble[MAX_BLE_DEVICES];
}ink_list_ble_data_t;


/*******************************************************
 * FUNCTIONS
********************************************************/

/**
 * @brief Finds the index of a device by MAC address.
 *
 * This function searches for a device with a specific MAC address within
 * the given array of devices.
 *
 * @param datos_guardados Array of stored device information.
 * @param mac_addr MAC address to search for.
 * @return Index of the device in the array, or -1 if not found.
 */
int Inkbird_mac_index(ink_sens_ble_t datos_guardados[MAX_BLE_DEVICES], esp_bd_addr_t mac_addr);



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


//char *Inkbird_trama_Mqtt(inkbird_sample *data, char *buffer);


int ink_esp_bd_addr__to__string(esp_bd_addr_t addr, char* str);

int ink_esp_bd_addr__to__string_1(esp_bd_addr_t addr, char* str);

/**
 * Convierte una cadena de texto que representa una dirección MAC a esp_bd_addr_t.
 *
 * Esta función toma una cadena de texto que representa una dirección MAC en formato
 * estándar (separada por dos puntos) y la convierte en un objeto esp_bd_addr_t,
 * que es una representación de la dirección MAC en el firmware de ESP32.
 *
 * @param mac_str La cadena de texto que representa la dirección MAC.
 * @param mac_addr Puntero a un objeto esp_bd_addr_t donde se almacenará la dirección MAC.
 * @return Devuelve 1 si la conversión fue exitosa, 0 si hubo un error.
 */
int ink_string__to__esp_bd_addr(const char *mac_str, esp_bd_addr_t *mac_addr);

int ink_check_addr_scan(esp_bd_addr_t addr1, ink_list_ble_addr_t* list_ble_addr, int* idx);

void ink_init_list_ble_addr(ink_list_ble_addr_t* list_ble_scan);
void ink_print_list_ble_addr(ink_list_ble_addr_t list_ble);

void ink_concat_lit_ble(ink_list_ble_addr_t list_ble, char* buffer);

/**
 * Comprueba si una dirección BLE dada existe en una lista de direcciones BLE.
 *
 * Esta función toma una dirección BLE `addr1` y una lista de direcciones BLE `list_ble_addr`,
 * y verifica si la dirección `addr1` existe en la lista.
 *
 * @param addr1 La dirección BLE que se desea buscar en la lista.
 * @param list_ble_addr La lista de direcciones BLE en la que se realizará la búsqueda.
 * @return Devuelve 1 si la dirección BLE `addr1` se encuentra en la lista `list_ble_addr`,
 *         y devuelve 0 si no se encuentra.
 */
int ink_check_exist_ble(esp_bd_addr_t addr1, ink_list_ble_addr_t list_ble_addr);


/**
 * Comprueba si una dirección BLE dada está presente en una lista de direcciones BLE y devuelve su índice.
 *
 * Esta función toma una dirección BLE `addr1`, una lista de direcciones BLE `list_ble_addr`,
 * y un puntero a un entero `idx`. Busca la dirección `addr1` en la lista y, si se encuentra, 
 * asigna su índice a `idx`.
 *
 * @param addr1 La dirección BLE que se desea buscar en la lista.
 * @param list_ble_addr La lista de direcciones BLE en la que se realizará la búsqueda.
 * @param idx Puntero a un entero donde se almacenará el índice de la dirección BLE encontrada.
 * @return Devuelve 1 si la dirección BLE `addr1` se encuentra en la lista `list_ble_addr`,
 *         y asigna su índice a `idx`. Devuelve 0 si la dirección BLE no se encuentra.
 */
int ink_check_indx_ble(esp_bd_addr_t addr1, ink_list_ble_addr_t list_ble_addr, int* idx);

/**
 * Verifica si la temperatura medida está dentro de los límites configurados.
 *
 * Esta función toma datos de un sensor BLE, verifica si la configuración de temperatura está activada,
 * y luego compara la temperatura medida con los límites configurados.
 *
 * @param ble_data Los datos del sensor BLE que incluyen la configuración de temperatura y la temperatura medida.
 * @return Devuelve un código de alerta dependiendo de si la temperatura está dentro de los límites configurados o no.
 *         - ALERT_BLE_DEACTIVE si la configuración de temperatura no está activada o si la temperatura medida se encuentra dentro de los límites.
 *         - ALERT_BLE_TMAX si la temperatura medida supera el límite máximo configurado.
 *         - ALERT_BLE_TMIN si la temperatura medida está por debajo del límite mínimo configurado.
 */
int ink_check_limit_temp(ink_sens_ble_t ble_data);

#endif // INKBIRD_BLE_H
