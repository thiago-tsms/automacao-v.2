#ifndef JSON_PARSE
#define JSON_PARSE

#include <string.h>
#include <stdlib.h>
#include "cJSON.h"

#include "data-types.h"

const char *TAG_JSON = "json-parse";

  /* Converte os dados para ser enviados em uma String JSON */
char* json_serialize(data_json_t *data){
  char *json_string = NULL;
  cJSON *json_object = cJSON_CreateObject();

  char ID[4], MODO[4];

  itoa(data->id, ID, 10);
  cJSON_AddStringToObject(json_object, "ID", (char *)ID);

  if(data->id == json_id_t::L1 || data->id == json_id_t::L2){
    cJSON_AddBoolToObject(json_object, "STATUS", data->status);

  } else if(data->id == json_id_t::L3){
    itoa(data->mode, MODO, 10);
    cJSON_AddBoolToObject(json_object, "STATUS", data->status);
    cJSON_AddStringToObject(json_object, "MODO", MODO);
  }

  json_string = cJSON_Print(json_object);
	cJSON_Delete(json_object);

  return json_string;
}

/*void json_deserialize(char *json_string){
  cJSON *json_object = cJSON_Parse(json_string);

  char *buffer = NULL;
  int esc = 0;
  
  if(cJSON_GetObjectItem(json_object, "ID")) {
		buffer = cJSON_GetObjectItem(json_object, "ID")->valuestring;
    esc = atoi(buffer);
	}

  if(cJSON_GetObjectItem(json_object, "bool")) {
		bool_teste = cJSON_GetObjectItem(json_object, "bool")->valueint;
	}

  ESP_LOGI(TAG_WIFI_MANAGER, "string=%s || boll=%d", buffer, esc);

  cJSON_Delete(json_object);
}*/

void json_string_free(char *json_string){
  cJSON_free(json_string);
}

#endif