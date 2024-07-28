#include "driver/i2c_master.h"
#include "driver/gpio.h"
#include "esp_err.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include <cstring>

#define ACK_CHECK_EN 0x1                        
#define ACK_CHECK_DIS 0x0                      

//static const char *TAG = "i2c-master";

typedef struct {
    gpio_num_t sda;
    gpio_num_t scl;
    uint8_t slaveAddr;
} i2c_master_config;

// DO 20 bits
// DI 24 bits

/*
SLAVE DOUT data format:
{0xBB, 0x0X, 0xXX} 
0xBB: first byte to indicate DO cmd
0x0X: LSB 2 bits for DO 9 and 8
0xXX: DO bit 7 to 0
*/

class shoalbot_master_i2c {
    public:
        shoalbot_master_i2c(i2c_master_config* conf);
        esp_err_t begin();
        esp_err_t i2c_send_DO(uint8_t* data);
        uint32_t read_di();
//        esp_err_t cntrl_BMSpass(uint8_t data); // 0xCC to pass BMS, Pass to slave
//        bool check_BATSW();
    
    private:
        i2c_master_bus_config_t i2c_mst_config = {};
        i2c_master_bus_handle_t i2c_mst_handle;
        i2c_device_config_t ESP_slave = {};
        i2c_master_dev_handle_t i2c_master_handle;
        i2c_master_event_callbacks_t cbs;
        esp_err_t ret;

        uint8_t DI_data[3] = {};
        uint8_t DI_cmd[1] = {0xFF}; // indicating to the slave to send DI data
        uint8_t batSW_cmd[1] = {0xDD}; // indicating to the slave to send BATSW data

//        uint8_t batSW = 0;

        uint32_t DI_fromSlave = 0;
        uint8_t diCnt = 0;
};