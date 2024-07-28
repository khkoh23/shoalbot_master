#ifndef AMIP4K_SPI_H
#define AMIP4K_SPI_H

#include "driver/spi_master.h"
#include "register.h"
#include "esp_log.h"
#include <iostream>
#include <inttypes.h>

#define WRA 0b1000 // Write Address (0x8+address) 0b1000
#define WRD 0b1010 // Write Data (0xA+data) 0b1010
#define RD0 0b1100 // Read bytes 0 + 1 (2LSB) (0xC+address) 0b1100
#define RD1 0b1110 // Read Bytes 2 + 3 (2MSB) (0xE) 0b1110
#define NOP 0b0000 // Output read Register 
// #define HWA 0b0000

// AM-IP4k SPI WORD FORMAT = 4 bit OP-CODE + 4 bit ADDRESS + 8 bit DATA

// static const char *TAG1 = "IC_SPI";IC

typedef struct {
    int miso;
    int mosi;
    int sclk;
    int cs;
    uint8_t hwa;
    bool init_bus; // to avoid re-initialization of bus
} amip4k_spi_config;

enum InterpolationRate : uint8_t { // interpolation rate
    rate_4096 = 0b10000,
    rate_4000 = 0b00000,
    rate_3200 = 0b01000,
    rate_2560 = 0b11000,
    rate_2048 = 0b10001,
    rate_2000 = 0b00001,
    rate_1600 = 0b01001,
    rate_1280 = 0b11001,
    rate_1024 = 0b10010,
    rate_1000 = 0b00010,
    rate_800 = 0b01010,
    rate_640 = 0b11010,
    rate_512 = 0b10011,
    rate_500 = 0b00011,
    rate_400 = 0b01011,
    rate_320 = 0b11011,
    rate_256 = 0b10100,
    rate_250 = 0b00100,
    rate_200 = 0b01100,
    rate_160 = 0b11100,
    rate_128 = 0b10101,
    rate_125 = 0b00101,
    rate_100 = 0b01101,
    rate_80 = 0b11101,
    rate_64 = 0b10110,
    rate_50 = 0b01110,
    rate_40 = 0b11110,
    rate_32 = 0b10111, // bit 0~2 used for IR_SUM calc to set rate under 20
    rate_25 = 0b01111,
    rate_20 = 0b11111,
    // IR_SUM(3:0) = IR(2:0) + IRDiv2(2:0)
    rate_16 = 0b001, // IRDiv2 for IR_SUM calc
    rate_8 = 0b010, // IRDiv2 for IR_SUM calc
    rate_4 = 0b011, // IRDiv2 for IR_SUM calc
};

/*SPI READ 32 bit Seq (OP-Code(4) + HWA(4) + DATA)
--------RD0 + HWA + addr -> RD1 +  HWA + 0x00 -> NOP + HWA + 0x00--------
*/

/*SPI Write 8 bit Seq (OP-Code(4) + HWA(4) + DATA)
--------WRA + HWA + addr ->  -> WRD + HWA + DATA--------
//////--REPEAT ABOVE SEQUENCE FOR 32 bit WRITING--//////
*/

class amip4k_spi {
    public:
        amip4k_spi(amip4k_spi_config *spi_config);
        esp_err_t begin();
        esp_err_t readSTAT();
        int32_t readMVAL();
        int32_t readCNT();
        int32_t readPOSIT();
        esp_err_t rate_conf(InterpolationRate rate);
        esp_err_t reset_cnt();

    private:
        uint8_t HWA = 0b0000;
        spi_bus_config_t busESP = {}; // SPI bus configuration
        spi_device_interface_config_t IC_dev = {}; // SPI slave configuration
        spi_device_handle_t handle; 
        spi_transaction_t t = {};
        uint16_t sendbuf[1] = {};
        uint16_t recvbuf[1] = {};
        int32_t MVAL = 0;
        int32_t CNT = 0;
        int32_t POSIT = 0;
        // CFG SET TO InterpolationRate 4 (DEFAULT FOR OUR CASE)
        uint8_t CFG1[4] = {0x17, 0x09, 0xFF, 0x00}; // stored from LSB to MSB
        uint8_t CFG2[4] = {0x66, 0x02, 0x80, 0xC5};
        uint8_t CFG3[4] = {0x00, 0x00, 0x00, 0x00};

        bool bus_init = 0;

        esp_err_t read_spi(uint8_t reg, uint8_t op_code = 0);
        esp_err_t write_spi(uint8_t reg, uint8_t op_code);
        esp_err_t write_CFG1();
        esp_err_t write_CFG2();
        esp_err_t write_CFG3(bool abSwitch = 0);
};


#endif // AMIP4K_SPI_H