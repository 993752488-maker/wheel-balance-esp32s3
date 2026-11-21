#pragma once
#include "esp_http_server.h"
esp_err_t product_http_register(httpd_handle_t server,const char *boot_id);
