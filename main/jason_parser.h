

#ifndef _JSON_PARSER_H_
#define _JSON_PARSER_H_

#include <stdio.h>
#include <inttypes.h>
#include <string.h>
#include <math.h>
#include <dirent.h>

#include "inkbird_ble.h"
#include "main.h"

int js_modem_to_str(const modem_gsm_t modem, char* buffer);

int js_record_data_ble(ink_ble_report_t data, char *buffer);



#endif /**/