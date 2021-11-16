/*
  cJSON_AddNumberToObject apresenta erro, 
  por isso tudo está sendo feito com String
*/
char* json_serialize(data_json *data){
  char *json_string = NULL;
  cJSON *json_object = cJSON_CreateObject();

  char ID[4], LD[2];

  itoa(data->ID, &ID, 10);
  itoa(data->LD, &LD, 10);

  cJSON_AddStringToObject(json_object, "ID", (char *)ID);
  cJSON_AddStringToObject(json_object, "LD", (char *)LD);

  json_string = cJSON_Print(json_object);
	cJSON_Delete(json_object);

  return json_string;
}

void json_deserialize(char *json_string){
  cJSON *json_object = cJSON_Parse(json_string);

  char *buffer = NULL;
  int esc;
  
  if(cJSON_GetObjectItem(json_object, "ID")) {
		buffer = cJSON_GetObjectItem(json_object, "ID")->valuestring;
    esc = atoi(buffer);
	}

  /*if(cJSON_GetObjectItem(json_object, "bool")) {
		bool_teste = cJSON_GetObjectItem(json_object, "bool")->valueint;
	}*/

  ESP_LOGI(TAG_WIFI_MANAGER, "string=%s || boll=%d", string_teste, bool_teste);

  cJSON_Delete(json_object);
}

void json_string_free(char *json_string){
  cJSON_free(json_string);
}
