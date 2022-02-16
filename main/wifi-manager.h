#ifndef WIFI_MANAGER
#define WIFI_MANAGER

#define WIFI_MANAGER_SSID_SERVER "WiFi Manager"

const char *TAG_WIFI_MANAGER = "wifi-manager";


  /* Escopo de funções */
void wifi_manager_get_ssid(char *ssid);
void wifi_manager_get_password(char *password);
void wifi_manager_get_ip(uint32_t *ip);
void wifi_manager_get_gateway(uint32_t *gateway);
void wifi_manager_get_netmask(uint32_t *netmask);
void wifi_manager_get_port(uint16_t *port);


  /* Funções */
void wifi_manager_get_ssid(char *ssid){

}

void wifi_manager_get_password(char *password){

}

void wifi_manager_get_ip(uint32_t *ip){

}

void wifi_manager_get_gateway(uint32_t *gateway){

}

void wifi_manager_get_netmask(uint32_t *netmask){

}

void wifi_manager_get_port(uint16_t *port){

}
#endif