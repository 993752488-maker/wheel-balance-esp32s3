# Dependency discovery precedes Kconfig evaluation in ESP-IDF. Keep the
# requirements unconditional; diagnostic_web.c supplies disabled stubs.
idf_component_register(SRCS "diagnostic_web.c" "product_http.c" "capture_export.c" INCLUDE_DIRS "include"
    PRIV_REQUIRES product_session json acquisition optical_ref serial_console board_config raw_capture
        esp_timer esp_app_format esp_hw_support esp_wifi esp_event esp_netif
        esp_http_server esp_http_client nvs_flash freertos heap
    EMBED_TXTFILES "web/index.html" "web/product.js" "web/product.css" "web/speed.js")
