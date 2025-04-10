idf_component_register(SRCS "TimeTableCode.c" "file_server.c" "ds3231.c" "i2cdev.c"
                    INCLUDE_DIRS "."
                    REQUIRES esp_wifi fatfs freertos nvs_flash esp_http_client esp_http_server esp_https_ota esp_https_server esp-tls spiffs esp_http_server esp_netif vfs driver ssd1306 protocol_examples_common
                    EMBED_FILES "upload_script.html")
                    spiffs_create_partition_image(storage ../partition FLASH_IN_PROJECT)
