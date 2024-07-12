@echo off
setlocal

set SDK_PATH=C:\Users\kakar\AppData\Local\Arduino15\packages\esp32\tools\esp32-arduino-libs\idf-release_v5.1-bd2b9390ef
set PROJECT_PATH=C:\Users\kakar\Documents\Arduino\libraries\IoTA_Basic

REM echo Compiling IoTA_Basic.cpp with the following include paths:
REM echo -IC:\Users\kakar\AppData\Local\Arduino15\packages\esp32\hardware\esp32\3.0.2\cores\esp32
REM echo -IC:\Users\kakar\AppData\Local\Arduino15\packages\esp32\hardware\esp32\3.0.2\variants\esp32
REM echo -I%SDK_PATH%\esp32\include
REM echo -I%SDK_PATH%\esp32\include\app_trace\include
REM echo -I%SDK_PATH%\esp32\include\app_update\include
REM echo -I%SDK_PATH%\esp32\include\bootloader_support\include
REM echo -I%SDK_PATH%\esp32\include\bootloader_support\bootloader_flash\include
REM echo -I%SDK_PATH%\esp32\include\bootloader_support\bootloader_flash\include\esp_private
REM echo -I%SDK_PATH%\esp32\include\coap\include
REM echo -I%SDK_PATH%\esp32\include\console\include
REM echo -I%SDK_PATH%\esp32\include\driver\include
REM echo -I%SDK_PATH%\esp32\include\esp-tls\include
REM echo -I%SDK_PATH%\esp32\include\esp_adc_cal\include
REM echo -I%SDK_PATH%\esp32\include\esp_app_format\include
REM echo -I%SDK_PATH%\esp32\include\esp_common\include
REM echo -I%SDK_PATH%\esp32\include\esp_event\include
REM echo -I%SDK_PATH%\esp32\include\esp_http_client\include
REM echo -I%SDK_PATH%\esp32\include\esp_http_server\include
REM echo -I%SDK_PATH%\esp32\include\esp_https_ota\include
REM echo -I%SDK_PATH%\esp32\include\esp_ipc\include
REM echo -I%SDK_PATH%\esp32\include\esp_local_ctrl\include
REM echo -I%SDK_PATH%\esp32\include\esp_netif\include
REM echo -I%SDK_PATH%\esp32\include\esp_phy\include
REM echo -I%SDK_PATH%\esp32\include\esp_pm\include
REM echo -I%SDK_PATH%\esp32\include\esp_ringbuf\include
REM echo -I%SDK_PATH%\esp32\include\esp_rom\include
REM echo -I%SDK_PATH%\esp32\include\esp_sleep\include
REM echo -I%SDK_PATH%\esp32\include\esp_timer\include
REM echo -I%SDK_PATH%\esp32\include\esp_websocket_client\include
REM echo -I%SDK_PATH%\esp32\include\esp_wifi\include
REM echo -I%SDK_PATH%\esp32\include\esp_system\include
REM echo -I%SDK_PATH%\esp32\include\esp_hw_support\include
REM echo -I%SDK_PATH%\esp32\include\soc\esp32\include\soc
REM echo -I%SDK_PATH%\esp32\include\espcoredump\include
REM echo -I%SDK_PATH%\esp32\include\expat\include
REM echo -I%SDK_PATH%\esp32\include\freertos\include
REM echo -I%SDK_PATH%\esp32\include\hal\include
REM echo -I%SDK_PATH%\esp32\include\heap\include
REM echo -I%SDK_PATH%\esp32\include\json\include
REM echo -I%SDK_PATH%\esp32\include\log\include
REM echo -I%SDK_PATH%\esp32\include\lwip\include
REM echo -I%SDK_PATH%\esp32\include\mbedtls\include
REM echo -I%SDK_PATH%\esp32\include\mdns\include
REM echo -I%SDK_PATH%\esp32\include\nvs_flash\include
REM echo -I%SDK_PATH%\esp32\include\pthread\include
REM echo -I%SDK_PATH%\esp32\include\spi_flash\include
REM echo -I%SDK_PATH%\esp32\include\tcp_transport\include
REM echo -I%SDK_PATH%\esp32\include\tcpip_adapter\include
REM echo -I%SDK_PATH%\esp32\include\ulp\include
REM echo -I%SDK_PATH%\esp32\include\vfs\include
REM echo -I%SDK_PATH%\esp32\include\wear_levelling\include
REM echo -I%SDK_PATH%\esp32\include\wifi_provisioning\include
REM echo -I%SDK_PATH%\esp32\include\xtensa\include
REM echo -I%PROJECT_PATH%

g++ -c %PROJECT_PATH%\IoTA_Basic.cpp -o %PROJECT_PATH%\IoTA_Basic.o ^
    -IC:\Users\kakar\AppData\Local\Arduino15\packages\esp32\hardware\esp32\3.0.2\cores\esp32 ^
    -IC:\Users\kakar\AppData\Local\Arduino15\packages\esp32\hardware\esp32\3.0.2\variants\esp32 ^
    -I%SDK_PATH%\esp32\include ^
    -I%SDK_PATH%\esp32\include\app_trace\include ^
    -I%SDK_PATH%\esp32\include\app_update\include ^
    -I%SDK_PATH%\esp32\include\bootloader_support\include ^
    -I%SDK_PATH%\esp32\include\bootloader_support\bootloader_flash\include ^
    -I%SDK_PATH%\esp32\include\bootloader_support\bootloader_flash\include\esp_private ^
    -I%SDK_PATH%\esp32\include\coap\include ^
    -I%SDK_PATH%\esp32\include\console\include ^
    -I%SDK_PATH%\esp32\include\driver\include ^
    -I%SDK_PATH%\esp32\include\esp-tls\include ^
    -I%SDK_PATH%\esp32\include\esp_adc_cal\include ^
    -I%SDK_PATH%\esp32\include\esp_app_format\include ^
    -I%SDK_PATH%\esp32\include\esp_common\include ^
    -I%SDK_PATH%\esp32\include\esp_event\include ^
    -I%SDK_PATH%\esp32\include\esp_http_client\include ^
    -I%SDK_PATH%\esp32\include\esp_http_server\include ^
    -I%SDK_PATH%\esp32\include\esp_https_ota\include ^
    -I%SDK_PATH%\esp32\include\esp_ipc\include ^
    -I%SDK_PATH%\esp32\include\esp_local_ctrl\include ^
    -I%SDK_PATH%\esp32\include\esp_netif\include ^
    -I%SDK_PATH%\esp32\include\esp_phy\include ^
    -I%SDK_PATH%\esp32\include\esp_pm\include ^
    -I%SDK_PATH%\esp32\include\esp_ringbuf\include ^
    -I%SDK_PATH%\esp32\include\esp_rom\include ^
    -I%SDK_PATH%\esp32\include\esp_sleep\include ^
    -I%SDK_PATH%\esp32\include\esp_timer\include ^
    -I%SDK_PATH%\esp32\include\esp_websocket_client\include ^
    -I%SDK_PATH%\esp32\include\esp_wifi\include ^
    -I%SDK_PATH%\esp32\include\esp_system\include ^
    -I%SDK_PATH%\esp32\include\esp_hw_support\include ^
    -I%SDK_PATH%\esp32\include\soc\esp32\include\soc ^ 
    -I%SDK_PATH%\esp32\include\espcoredump\include ^
    -I%SDK_PATH%\esp32\include\expat\include ^
    -I%SDK_PATH%\esp32\include\freertos\include ^
    -I%SDK_PATH%\esp32\include\hal\include ^
    -I%SDK_PATH%\esp32\include\heap\include ^
    -I%SDK_PATH%\esp32\include\json\include ^
    -I%SDK_PATH%\esp32\include\log\include ^
    -I%SDK_PATH%\esp32\include\lwip\include ^
    -I%SDK_PATH%\esp32\include\mbedtls\include ^
    -I%SDK_PATH%\esp32\include\mdns\include ^
    -I%SDK_PATH%\esp32\include\nvs_flash\include ^
    -I%SDK_PATH%\esp32\include\pthread\include ^
    -I%SDK_PATH%\esp32\include\spi_flash\include ^
    -I%SDK_PATH%\esp32\include\tcp_transport\include ^
    -I%SDK_PATH%\esp32\include\tcpip_adapter\include ^
    -I%SDK_PATH%\esp32\include\ulp\include ^
    -I%SDK_PATH%\esp32\include\vfs\include ^
    -I%SDK_PATH%\esp32\include\wear_levelling\include ^
    -I%SDK_PATH%\esp32\include\wifi_provisioning\include ^
    -I%SDK_PATH%\esp32\include\xtensa\include ^
    -I%SDK_PATH%\esp32\qio_qspi\include\
    REM -I%PROJECT_PATH%

if %ERRORLEVEL% NEQ 0 (
    echo Compilation failed
    exit /b %ERRORLEVEL%
)

echo Compilation and linking successful
endlocal
