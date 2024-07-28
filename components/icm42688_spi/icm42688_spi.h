#ifndef ICM42688_SPI_H
#define ICM42688_SPI_H

#include "register.h"
#include "esp_err.h"
#include "driver/spi_master.h"
#include "esp_log.h"
#include <driver/gpio.h>
#include "esp_system.h"
#include <string>
#include <math.h>

typedef struct {
    int miso;
    int mosi;
    int sclk;
    int cs;
    bool init_bus;
} icm42688_spi_config;

enum GyroFSR : uint8_t { // Full Scale Range gyro
    dps2000 = 0x00,
    dps1000 = 0x01,
    dps500 = 0x02,
    dps250 = 0x03,
    dps125 = 0x04,
    dps62_5 = 0x05,
    dps31_25 = 0x06,
    dps15_625 = 0x07
};
enum AccelFSR : uint8_t { // Full Scale Range accel
    g16 = 0x00,
    g8 = 0x01,
    g4 = 0x02,
    g2 = 0x03
};
enum ODR : uint8_t { // Output Data Rate
    odr32k = 0x01, // LN mode
    odr16k = 0x02, // LN mode
    odr8k = 0x03, // LN mode
    odr4k = 0x04, // LN mode
    odr2k = 0x05, // LN mode
    odr1k = 0x06, // LN mode (default)
    odr200 = 0x07, // LP, LN mode
    odr100 = 0x08, // LP, LN mode
    odr50 = 0x09, // LP, LN mode
    odr25 = 0x0A, // LP, LN mode
    odr12a5 = 0x0B, // LP, LN mode
    odr6a25 = 0x0C, // LP mode
    odr3a125 = 0x0D, // LP mode 
    odr1a5625 = 0x0E, // LP mode
    odr500 = 0x0F, // LP, LN mode
};

enum notch_bandwidth : uint8_t {
    bw1449 = 0x00,
    bw680 = 0x01,
    bw329 = 0x02,
    bw162 = 0x03,
    bw80 = 0x04,
    bw40 = 0x05,
    bw20 = 0x06,
    bw10 = 0x07,
};

enum UI_order : uint8_t {
    order1 = 0b00,
    order2 = 0b01,
    order3 = 0b10,

};


class icm42688_spi {
    public:
        icm42688_spi(icm42688_spi_config *spi_config);
        esp_err_t begin();
        esp_err_t reset();
        esp_err_t set_gyro_fsr(GyroFSR fsr);
        esp_err_t set_accel_fsr(AccelFSR fsr);
        esp_err_t set_accODR(ODR odr);
        esp_err_t set_gyroODR(ODR odr);
        double get_accel_x(uint8_t ac_flg = 0);
        double get_accel_y(uint8_t ac_flg = 0);
        double get_accel_z(uint8_t ac_flg = 0);
        double get_gyro_x(uint8_t gb_flg = 0);
        double get_gyro_y(uint8_t gb_flg = 0);
        double get_gyro_z(uint8_t gb_flg = 0);

    private:
        esp_err_t ret;
        spi_device_handle_t handle;
        spi_bus_config_t buscfg = {};
        spi_device_interface_config_t devcfg = {};
        spi_transaction_t t = {};
        uint8_t sendbuf[1] = {0};
        uint8_t recvbuf[1] = {0};
        float gyro_fsr = 2000.0;
        float accel_fsr = 16.0;
        double gyro_bias[3] = {};
        double acc_bias[3] = {};
        bool bus_init = 0;
        uint8_t AAF_bitShift = 0;
        uint16_t AAF_deltSqr = 0;
        

        esp_err_t read_spi(uint8_t reg);
        esp_err_t write_spi(uint8_t reg, uint8_t data, uint8_t len);
        esp_err_t who_am_i();
        esp_err_t set_nf_aaf(bool nf_mode, bool aaf_mode); // mode = 0: disable, mode = 1: enable
        esp_err_t set_gyroNF_freq(double freq); // 1kHz <= freq <= 3kHz
        esp_err_t set_gyroNF_bw(notch_bandwidth bw);
        esp_err_t set_aaf_bandwidth(uint8_t bandwidth); 
        esp_err_t set_ui_filter(UI_order filter_order, uint8_t filter_index);

        int16_t get_ax0();
        int16_t get_ax1();
        int16_t get_ay0();
        int16_t get_ay1();
        int16_t get_az0();
        int16_t get_az1();
        int16_t get_gx0();
        int16_t get_gx1();
        int16_t get_gy0();
        int16_t get_gy1();
        int16_t get_gz0();
        int16_t get_gz1();

        const double PI = 3.14159265359;
};


#endif // ICM42688_SPI_H