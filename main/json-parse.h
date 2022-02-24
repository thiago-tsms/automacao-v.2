#ifndef JSON_PARSE
#define JSON_PARSE

#include <string.h>
#include <stdlib.h>
#include "cJSON.h"

#include "esp_log.h"

#include "data-types.h"


  /* Identificador dos campos JSON */
#define JSON_ID "ID"
#define JSON_L1 "L1"
#define JSON_L2 "L2"
#define JSON_LED "LED"
#define JSON_MODE "MODE"
#define JSON_FX "FX"
#define JSON_AMP_R "AMP_R"
#define JSON_PER_R "PER_R"
#define JSON_DES_R "DES_R"
#define JSON_AMP_G "AMP_G"
#define JSON_PER_G "PER_G"
#define JSON_DES_G "DES_G"
#define JSON_AMP_B "AMP_B"
#define JSON_PER_B "PER_B"
#define JSON_DES_B "DES_B"


  /* TAG de Log */
const char *TAG_JSON = "json-parse";


  /* Converte os dados para ser enviados em uma String JSON */
char* json_serialize(data_json_t *data){
  char *json_string = NULL;
  cJSON *json_object = cJSON_CreateObject();
  char data_str[32];

    // ID
  itoa(data->id, data_str, 10);
  cJSON_AddStringToObject(json_object, JSON_ID, (char *)data_str);

    // L1
  if(data->mask & mask_json_t::L1){
    itoa(data->ls.l1, data_str, 10);
    cJSON_AddStringToObject(json_object, JSON_L1, (char *)data_str);
  }

    // L2
  if(data->mask & mask_json_t::L2){
    itoa(data->ls.l2, data_str, 10);
    cJSON_AddStringToObject(json_object, JSON_L2, (char *)data_str);
  }

    // LED
  if(data->mask & mask_json_t::LED){
    itoa(data->ls.led, data_str, 10);
    cJSON_AddStringToObject(json_object, JSON_LED, (char *)data_str);
  }

    // MODE
  if(data->mask & mask_json_t::MODE){
    itoa(data->ls.mode, data_str, 10);
    cJSON_AddStringToObject(json_object, JSON_MODE, (char *)data_str);
  }

    // FX
  if(data->mask & mask_json_t::FX){
    itoa(data->rgb.fx, data_str, 10);
    cJSON_AddStringToObject(json_object, JSON_FX, (char *)data_str);
  }

    // AMP R
  if(data->mask & mask_json_t::AMP_R){
    itoa(data->rgb.r.amp, data_str, 10);
    cJSON_AddStringToObject(json_object, JSON_AMP_R, (char *)data_str);
  }

    // PER R
  if(data->mask & mask_json_t::PER_R){
    itoa(data->rgb.r.per, data_str, 10);
    cJSON_AddStringToObject(json_object, JSON_PER_R, (char *)data_str);
  }

    // DES R
  if(data->mask & mask_json_t::DES_R){
    itoa(data->rgb.r.des, data_str, 10);
    cJSON_AddStringToObject(json_object, JSON_DES_R, (char *)data_str);
  }

    // AMP G
  if(data->mask & mask_json_t::AMP_G){
    itoa(data->rgb.g.amp, data_str, 10);
    cJSON_AddStringToObject(json_object, JSON_AMP_G, (char *)data_str);
  }

    // PER G
  if(data->mask & mask_json_t::PER_G){
    itoa(data->rgb.g.per, data_str, 10);
    cJSON_AddStringToObject(json_object, JSON_PER_G, (char *)data_str);
  }

    // DES G
  if(data->mask & mask_json_t::DES_G){
    itoa(data->rgb.g.des, data_str, 10);
    cJSON_AddStringToObject(json_object, JSON_DES_G, (char *)data_str);
  }

    // AMP B
  if(data->mask & mask_json_t::AMP_B){
    itoa(data->rgb.b.amp, data_str, 10);
    cJSON_AddStringToObject(json_object, JSON_AMP_B, (char *)data_str);
  }

    // PER B
  if(data->mask & mask_json_t::PER_B){
    itoa(data->rgb.b.per, data_str, 10);
    cJSON_AddStringToObject(json_object, JSON_PER_B, (char *)data_str);
  }

    // DES B
  if(data->mask & mask_json_t::DES_B){
    itoa(data->rgb.b.des, data_str, 10);
    cJSON_AddStringToObject(json_object, JSON_DES_B, (char *)data_str);
  }

  json_string = cJSON_Print(json_object);
	cJSON_Delete(json_object);

  ESP_LOGV(TAG_JSON, "JSON Serializado");

  return json_string;
}

data_json_t json_deserialize(char *json_string){
  cJSON *json_object = cJSON_Parse(json_string);
  data_json_t data;
  char *buffer;

  data.mask = 0;
  
    // ID
  if(cJSON_GetObjectItem(json_object, JSON_ID)) {
		buffer = cJSON_GetObjectItem(json_object, JSON_ID)->valuestring;

    switch(atoi(buffer)){
      case 0: data.id = id_json_t::NO_ACTION; break;
      case 1: data.id = id_json_t::DATA_TRANSACTION; break;
    }

    data.mask += mask_json_t::ID; 
	}

    // L1
  if(cJSON_GetObjectItem(json_object, JSON_L1)) {
		buffer = cJSON_GetObjectItem(json_object, JSON_L1)->valuestring;
    data.ls.l1 = (bool)atoi(buffer);
    data.mask += mask_json_t::L1; 
	}

    // L2
  if(cJSON_GetObjectItem(json_object, JSON_L2)) {
		buffer = cJSON_GetObjectItem(json_object, JSON_L2)->valuestring;
    data.ls.l2 = (bool)atoi(buffer);
    data.mask += mask_json_t::L2; 
	}

    // LED
  if(cJSON_GetObjectItem(json_object, JSON_LED)) {
		buffer = cJSON_GetObjectItem(json_object, JSON_LED)->valuestring;
    data.ls.led = (bool)atoi(buffer);
    data.mask += mask_json_t::LED; 
	}

    // MODE
  if(cJSON_GetObjectItem(json_object, JSON_MODE)) {
		buffer = cJSON_GetObjectItem(json_object, JSON_MODE)->valuestring;
    data.ls.mode = (uint8_t)atoi(buffer);
    data.mask += mask_json_t::MODE; 
	}

    // FX
  if(cJSON_GetObjectItem(json_object, JSON_FX)) {
		buffer = cJSON_GetObjectItem(json_object, JSON_FX)->valuestring;
    data.rgb.fx = (bool)atoi(buffer);
    data.mask += mask_json_t::FX; 
	}

    // AMP_R
  if(cJSON_GetObjectItem(json_object, JSON_AMP_R)) {
		buffer = cJSON_GetObjectItem(json_object, JSON_AMP_R)->valuestring;
    data.rgb.r.amp = (uint16_t)atoi(buffer);
    data.mask += mask_json_t::AMP_R; 
	}

    // PER_R
  if(cJSON_GetObjectItem(json_object, JSON_PER_R)) {
		buffer = cJSON_GetObjectItem(json_object, JSON_PER_R)->valuestring;
    data.rgb.r.per = (uint16_t)atoi(buffer);
    data.mask += mask_json_t::PER_R; 
	}

    // DES_R
  if(cJSON_GetObjectItem(json_object, JSON_DES_R)) {
		buffer = cJSON_GetObjectItem(json_object, JSON_DES_R)->valuestring;
    data.rgb.r.des = (uint16_t)atoi(buffer);
    data.mask += mask_json_t::DES_R; 
	}

    // AMP_G
  if(cJSON_GetObjectItem(json_object, JSON_AMP_G)) {
		buffer = cJSON_GetObjectItem(json_object, JSON_AMP_G)->valuestring;
    data.rgb.g.amp = (uint16_t)atoi(buffer);
    data.mask += mask_json_t::AMP_G; 
	}

    // PER_G
  if(cJSON_GetObjectItem(json_object, JSON_PER_G)) {
		buffer = cJSON_GetObjectItem(json_object, JSON_PER_G)->valuestring;
    data.rgb.g.per = (uint16_t)atoi(buffer);
    data.mask += mask_json_t::PER_G; 
	}

    // DES_G
  if(cJSON_GetObjectItem(json_object, JSON_DES_G)) {
		buffer = cJSON_GetObjectItem(json_object, JSON_DES_G)->valuestring;
    data.rgb.g.des = (uint16_t)atoi(buffer);
    data.mask += mask_json_t::DES_G; 
	}

    // AMP_B
  if(cJSON_GetObjectItem(json_object, JSON_AMP_B)) {
		buffer = cJSON_GetObjectItem(json_object, JSON_AMP_B)->valuestring;
    data.rgb.b.amp = (uint16_t)atoi(buffer);
    data.mask += mask_json_t::AMP_B; 
	}

    // PER_B
  if(cJSON_GetObjectItem(json_object, JSON_PER_B)) {
		buffer = cJSON_GetObjectItem(json_object, JSON_PER_B)->valuestring;
    data.rgb.b.per = (uint16_t)atoi(buffer);
    data.mask += mask_json_t::PER_B; 
	}

    // DES_B
  if(cJSON_GetObjectItem(json_object, JSON_DES_B)) {
		buffer = cJSON_GetObjectItem(json_object, JSON_DES_B)->valuestring;
    data.rgb.b.des = (uint16_t)atoi(buffer);
    data.mask += mask_json_t::DES_B; 
	}

  cJSON_Delete(json_object);

  ESP_LOGV(TAG_JSON, "JSON Desserializado");

  return data;
}

void json_string_free(char *json_string){
  cJSON_free(json_string);
}

#endif