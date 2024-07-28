#include "icm42688_spi.h"

icm42688_spi::icm42688_spi(icm42688_spi_config *spi_config) {
    buscfg.mosi_io_num = spi_config->mosi;
    buscfg.miso_io_num = spi_config->miso;
    buscfg.sclk_io_num = spi_config->sclk;
    buscfg.quadwp_io_num = -1;
    buscfg.quadhd_io_num = -1;

    devcfg.command_bits = 8;
    devcfg.address_bits = 0;
    devcfg.dummy_bits = 0;
    devcfg.clock_speed_hz = 1000000; // 1 MHz
    devcfg.duty_cycle_pos = 128;
    devcfg.mode = 3;
    devcfg.spics_io_num = spi_config->cs;
    devcfg.cs_ena_posttrans = 0;
    devcfg.cs_ena_pretrans = 0; 
    devcfg.queue_size = 5;

    bus_init = spi_config->init_bus;
}   

esp_err_t icm42688_spi::read_spi(uint8_t reg) {
    t.length = 8;
    t.cmd = reg | 0x80;
    t.tx_buffer = 0;
    t.rx_buffer = recvbuf;
    ret = spi_device_transmit(handle, &t);
    return ret;
}

esp_err_t icm42688_spi::write_spi(uint8_t reg, uint8_t data, uint8_t len) {
    t.length = len * 8;
    t.cmd = reg;
    sendbuf[0] = data;
    t.tx_buffer = sendbuf;
    t.rx_buffer = 0;
    ret = spi_device_transmit(handle, &t);
    return ret;
}

esp_err_t icm42688_spi::begin() {
    if(!bus_init) {
        ret = spi_bus_initialize(SPI2_HOST, &buscfg, SPI_DMA_CH_AUTO);
        if(ret != ESP_OK) return ret;
    }
    ret = spi_bus_add_device(SPI2_HOST, &devcfg, &handle);
    if(ret != ESP_OK) return ret;
    ret = who_am_i();
    if(ret != ESP_OK) return ret;
    ret = write_spi(ICM42688reg::PWR_MGMT0, 0x0F, 2); // turn on gyro and accel sensors in LN mode
    if(ret != ESP_OK) return ret;
    vTaskDelay(5);
    ret = read_spi(ICM42688reg::PWR_MGMT0);
    //printf("Received data: 0x%02X\n", recvbuf[0]);
    set_accel_fsr(AccelFSR::g4); vTaskDelay(1);
    set_accODR(ODR::odr500); vTaskDelay(1);
    set_gyro_fsr(GyroFSR::dps250); vTaskDelay(1); //dps500
    set_gyroODR(ODR::odr500); vTaskDelay(1);
    for(int i = 0; i < 500; i++) {
        gyro_bias[0] += get_gyro_x(1);
        gyro_bias[1] += get_gyro_y(1);
        gyro_bias[2] += get_gyro_z(1);
        acc_bias[0] += get_accel_x(1);
        acc_bias[1] += get_accel_y(1);
        acc_bias[2] += get_accel_z(1);
    }
    gyro_bias[0] /= 500.0;
    gyro_bias[1] /= 500.0;
    gyro_bias[2] /= 500.0;
    acc_bias[0] /= 500.0;
    acc_bias[1] /= 500.0;
    acc_bias[2] /= 500.0;
    acc_bias[2] -= 9.81; 

    set_nf_aaf(1, 1);
    set_gyroNF_freq(1000.0);
    set_gyroNF_bw(notch_bandwidth::bw1449); //bw10
    set_aaf_bandwidth(63);
    set_ui_filter(UI_order::order3, 5);
    return ret;
}

esp_err_t icm42688_spi::reset() {
    ret = write_spi(ICM42688reg::DEVICE_CONFIG, 0x01, 2);
    return ret;
}

esp_err_t icm42688_spi::who_am_i() {
    ret = read_spi(ICM42688reg::WHO_AM_I);
    // printf("Received data: 0x%02X\n", recvbuf[0]);
    assert(recvbuf[0] == 0x47);
    return ret;
}

esp_err_t icm42688_spi::set_gyro_fsr(GyroFSR fsr) {
    read_spi(ICM42688reg::GYRO_CONFIG0);
    uint8_t reg = (fsr << 5) | (recvbuf[0] & 0x1f);
    ret = write_spi(ICM42688reg::GYRO_CONFIG0, reg, 2);
    switch(fsr) {
        case GyroFSR::dps2000:
            gyro_fsr = 2000.0;
            break;
        case GyroFSR::dps1000:
            gyro_fsr = 1000.0;
            break;
        case GyroFSR::dps500:
            gyro_fsr = 500.0;
            break;
        case GyroFSR::dps250:
            gyro_fsr = 250.0;
            break;
        case GyroFSR::dps125:
            gyro_fsr = 125.0;
            break;
        case GyroFSR::dps62_5:
            gyro_fsr = 62.5;
            break;
        case GyroFSR::dps31_25:
            gyro_fsr = 31.25;
            break;
        case GyroFSR::dps15_625:
            gyro_fsr = 15.625;
            break;
    }
    read_spi(ICM42688reg::GYRO_CONFIG0);
    // printf("Received data: 0x%02X\n", recvbuf[0]);
    return ret;
}

esp_err_t icm42688_spi::set_accel_fsr(AccelFSR fsr) {
    read_spi(ICM42688reg::ACCEL_CONFIG0);
    uint8_t reg = (fsr << 5) | (recvbuf[0] & 0x1f);
    ret = write_spi(ICM42688reg::ACCEL_CONFIG0, reg, 2);
    switch(fsr) {
        case AccelFSR::g16:
            accel_fsr = 16.0;
            break;
        case AccelFSR::g8:
            accel_fsr = 8.0;
            break;
        case AccelFSR::g4:
            accel_fsr = 4.0;
            break;
        case AccelFSR::g2:
            accel_fsr = 2.0;
            break;
    }
    read_spi(ICM42688reg::ACCEL_CONFIG0);
    // printf("Received data: 0x%02X\n", recvbuf[0]);
    return ret;
}

esp_err_t icm42688_spi::set_accODR(ODR odr) {
    read_spi(ICM42688reg::ACCEL_CONFIG0);
    uint8_t reg = (recvbuf[0] & 0xF0) | odr;
    ret = write_spi(ICM42688reg::ACCEL_CONFIG0, reg, 2);
    read_spi(ICM42688reg::ACCEL_CONFIG0);
    // printf("Received data: 0x%02X\n", recvbuf[0]);
    return ret;
}

esp_err_t icm42688_spi::set_gyroODR(ODR odr) {
    read_spi(ICM42688reg::GYRO_CONFIG0);
    uint8_t reg = (recvbuf[0] & 0xF0) | odr;
    ret = write_spi(ICM42688reg::GYRO_CONFIG0, reg, 2);
    read_spi(ICM42688reg::GYRO_CONFIG0);
    // printf("Received data: 0x%02X\n", recvbuf[0]);
    return ret;
}

esp_err_t icm42688_spi::set_nf_aaf(bool nf_mode, bool aaf_mode) {
    if(nf_mode && aaf_mode) {
        ret = write_spi(ICM42688reg::GYRO_CONFIG_STATIC2, 0x00, 1);
    }
    else if(nf_mode && !aaf_mode) {
        ret = write_spi(ICM42688reg::GYRO_CONFIG_STATIC2, 0x02, 1);
    }
    else if(!nf_mode && aaf_mode) {
        ret = write_spi(ICM42688reg::GYRO_CONFIG_STATIC2, 0x01, 1);
    }
    else {
        ret = write_spi(ICM42688reg::GYRO_CONFIG_STATIC2, 0x03, 1);

    }
    return ret;
}

esp_err_t icm42688_spi::set_gyroNF_freq(double freq) {
    int16_t NF_COSWZ = 0;
    uint8_t NF_COSWZ_SEL = 0;
    double coswz = cos(2 * M_PI * freq / 32.0);
    if(abs(coswz) <= 0.875) {
        NF_COSWZ = round(coswz * 256);
        NF_COSWZ_SEL = 0;
    }
    else {
        NF_COSWZ_SEL = 1;
        if(coswz > 0.875) {
            NF_COSWZ = round(8*(1-coswz)*256);
        }
        else if(coswz < -0.875) {
            NF_COSWZ = round(-8*(1+coswz)*256);
        }
    }
    uint8_t nf_coswz = NF_COSWZ >> 8;
    ret = write_spi(ICM42688reg::GYRO_CONFIG_STATIC6, NF_COSWZ, 1);
    ret = write_spi(ICM42688reg::GYRO_CONFIG_STATIC7, NF_COSWZ, 1);
    ret = write_spi(ICM42688reg::GYRO_CONFIG_STATIC8, NF_COSWZ, 1);
    uint8_t CONFIG_STATIC9 = 0x00;
    CONFIG_STATIC9 = (NF_COSWZ_SEL << 5) | (NF_COSWZ_SEL << 4) | (NF_COSWZ_SEL << 3) | (nf_coswz << 2) | (nf_coswz << 1) | nf_coswz;
    //printf("CONFIG_STATIC9: 0x%02X\n", CONFIG_STATIC9);
    ret = write_spi(ICM42688reg::GYRO_CONFIG_STATIC9, CONFIG_STATIC9, 1);
    ret = write_spi(ICM42688reg::GYRO_CONFIG_STATIC6, NF_COSWZ, 1);
    ret = write_spi(ICM42688reg::GYRO_CONFIG_STATIC7, NF_COSWZ, 1);
    ret = write_spi(ICM42688reg::GYRO_CONFIG_STATIC8, NF_COSWZ, 1);
    
    return ret;
}

esp_err_t icm42688_spi::set_gyroNF_bw(notch_bandwidth bw) {
    uint8_t set_bw = (bw<<4) | 0x01;
    ret = write_spi(ICM42688reg::GYRO_CONFIG_STATIC5, set_bw, 1);
    return ret;
}

esp_err_t icm42688_spi::set_aaf_bandwidth(uint8_t bandwidth) {
     // AAF_deltSqr = bandwidth * bandwidth;
    AAF_deltSqr = 3968;
    uint8_t accel_bw = (bandwidth << 1) | 0x00;
    ret = write_spi(ICM42688reg::GYRO_CONFIG_STATIC3, bandwidth, 1); //set gyro bandwidth for aaf
    if(ret != ESP_OK) return ret;
    ret = write_spi(ICM42688reg::ACCEL_CONFIG_STATIC2, accel_bw, 1); //set accel bandwidth for aaf
    if(ret != ESP_OK) return ret;
    ret = write_spi(ICM42688reg::GYRO_CONFIG_STATIC4, AAF_deltSqr, 1);
    if(ret != ESP_OK) return ret;
    ret = write_spi(ICM42688reg::ACCEL_CONFIG_STATIC3, AAF_deltSqr, 1);
    if(ret != ESP_OK) return ret;
    uint8_t AAF_DELTSQR = AAF_deltSqr >> 8;
    if(bandwidth == 1){
        AAF_bitShift = 15;
    } else if(bandwidth == 2){
        AAF_bitShift = 13;
    } else if(bandwidth == 3){
        AAF_bitShift = 12;
    } else if(bandwidth == 4){
        AAF_bitShift = 11;
    } else if(bandwidth == 5 || bandwidth == 6){
        AAF_bitShift = 10;
    } else if(bandwidth > 6 && bandwidth < 10){
        AAF_bitShift = 9;
    } else if(bandwidth > 9 && bandwidth < 14){
        AAF_bitShift = 8;
    } else if(bandwidth > 13 && bandwidth < 19){
        AAF_bitShift = 7;
    } else if(bandwidth > 18 && bandwidth < 27){
        AAF_bitShift = 6;
    } else if(bandwidth > 26 && bandwidth < 37){
        AAF_bitShift = 5;
    } else if(bandwidth > 36 && bandwidth < 53){
        AAF_bitShift = 4;
    } else if(bandwidth > 53 && bandwidth <= 63){
        AAF_bitShift = 3;
    }
    uint8_t AAF = 0x00;
    AAF = (AAF_bitShift << 4) | AAF_DELTSQR;
    ret = write_spi(ICM42688reg::GYRO_CONFIG_STATIC5, AAF, 1);
    if(ret != ESP_OK) return ret;
    ret = write_spi(ICM42688reg::ACCEL_CONFIG_STATIC4, AAF, 1);
    if(ret != ESP_OK) return ret;

    return ret;
}

esp_err_t icm42688_spi::set_ui_filter(UI_order filter_order, uint8_t filter_index) {
    ret = read_spi(ICM42688reg::GYRO_CONFIG1);
    if(ret != ESP_OK) return ret;
    // printf("Received data(GYRO UI): 0x%02X\n", recvbuf[0]);
    uint8_t UI_gyroOrder = recvbuf[0];
    UI_gyroOrder &= ~(0b11 << 2);
    UI_gyroOrder |= (filter_order & 0b11) << 2;
    // printf("UI_gyroOrder: 0x%02X\n", UI_gyroOrder);
    ret = write_spi(ICM42688reg::GYRO_CONFIG1, UI_gyroOrder, 1);
    if(ret != ESP_OK) return ret;
    ret = read_spi(ICM42688reg::GYRO_ACCEL_CONFIG0);
    if(ret != ESP_OK) return ret;
    // printf("Received data(GYRO ACCEL UI): 0x%02X\n", recvbuf[0]);
    uint8_t UI_gyroIndex = recvbuf[0];
    UI_gyroIndex &= ~(0b1111);
    UI_gyroIndex |= filter_index & 0b1111;
    // printf("UI_gyroIndex: 0x%02X\n", UI_gyroIndex);
    ret = write_spi(ICM42688reg::GYRO_ACCEL_CONFIG0, UI_gyroIndex, 1);
    if(ret != ESP_OK) return ret;

    ret = read_spi(ICM42688reg::ACCEL_CONFIG1);
    if(ret != ESP_OK) return ret;
    // printf("Received data(ACCEL UI): 0x%02X\n", recvbuf[0]); 
    uint8_t UI_accelOrder = recvbuf[0];
    UI_accelOrder &= ~(0b11 << 3);
    UI_accelOrder |= (filter_order & 0b11) << 3;
    // printf("UI_accelOrder: 0x%02X\n", UI_accelOrder);
    ret = write_spi(ICM42688reg::ACCEL_CONFIG1, UI_accelOrder, 1);
    if(ret != ESP_OK) return ret;
    ret = read_spi(ICM42688reg::GYRO_ACCEL_CONFIG0);
    if(ret != ESP_OK) return ret;
    // printf("Received data(GYRO ACCEL UI): 0x%02X\n", recvbuf[0]);
    uint8_t UI_accelIndex = recvbuf[0];
    UI_accelIndex &= ~(0b1111 << 4);
    UI_accelIndex |= (filter_index & 0b1111) << 4;
    // printf("UI_accelIndex: 0x%02X\n", UI_accelIndex);
    ret = write_spi(ICM42688reg::GYRO_ACCEL_CONFIG0, UI_accelIndex, 1);

    return ret;
}

double icm42688_spi::get_accel_x(uint8_t ac_flg) {
    read_spi(ICM42688reg::ACCEL_DATA_X0);
    int16_t accel_data_x0 = recvbuf[0];
    read_spi(ICM42688reg::ACCEL_DATA_X1);
    int16_t accel_data_x1 = recvbuf[0];
    int16_t accel_x_raw = (accel_data_x1 << 8) | accel_data_x0;
    double acc_x = (accel_x_raw * accel_fsr/32768.0) * 9.81;
    if(ac_flg == 0) {
        acc_x -= acc_bias[0];
    }
    // printf("Accel X: %f\n", acc_x);
    return acc_x;
}

double icm42688_spi::get_accel_y(uint8_t ac_flg) {
    read_spi(ICM42688reg::ACCEL_DATA_Y0);
    int16_t accel_data_y0 = recvbuf[0];
    read_spi(ICM42688reg::ACCEL_DATA_Y1);
    int16_t accel_data_y1 = recvbuf[0];
    int16_t accel_y_raw = (accel_data_y1 << 8) | accel_data_y0;
    double acc_y = (accel_y_raw * accel_fsr/32768.0) * 9.81;
    if(ac_flg == 0) {
        acc_y -= acc_bias[1];
    }
    // printf("Accel Y: %f\n", acc_y);
    return acc_y;
}

double icm42688_spi::get_accel_z(uint8_t ac_flg) {
    read_spi(ICM42688reg::ACCEL_DATA_Z0);
    int16_t accel_data_z0 = recvbuf[0];
    read_spi(ICM42688reg::ACCEL_DATA_Z1);
    int16_t accel_data_z1 = recvbuf[0];
    int16_t accel_z_raw = (accel_data_z1 << 8) | accel_data_z0;
    double acc_z = (accel_z_raw * accel_fsr/32768.0) * 9.81;
    if(ac_flg == 0) {
        acc_z -= acc_bias[2];
    }
    // printf("Accel Z: %f\n", acc_z);
    return acc_z;
}

double icm42688_spi::get_gyro_x(uint8_t gb_flg) {
    read_spi(ICM42688reg::GYRO_DATA_X0);
    uint8_t gyro_data_x0 = recvbuf[0];  
    read_spi(ICM42688reg::GYRO_DATA_X1);
    uint8_t gyro_data_x1 = recvbuf[0]; 
    int16_t gyro_x_raw = (gyro_data_x1 << 8) | gyro_data_x0;  
    double gyro_x = gyro_x_raw * (gyro_fsr / 32768.0) * (PI / 180); 
    if(gb_flg == 0) {
        gyro_x -= gyro_bias[0];
    } 
    // printf("Gyro X: %f\n", gyro_x);
    return gyro_x;
}

double icm42688_spi::get_gyro_y(uint8_t gb_flg) {
    read_spi(ICM42688reg::GYRO_DATA_Y0);
    int16_t gyro_data_y0 = recvbuf[0];  
    read_spi(ICM42688reg::GYRO_DATA_Y1);
    int16_t gyro_data_y1 = recvbuf[0];  
    int16_t gyro_y_raw = (gyro_data_y1 << 8) | gyro_data_y0;  
    double gyro_y = gyro_y_raw * (gyro_fsr / 32768.0)  * (PI / 180);  
    if(gb_flg == 0) {
        gyro_y -= gyro_bias[1];
    }
    // printf("Gyro Y: %f\n", gyro_y);
    return gyro_y;
}

double icm42688_spi::get_gyro_z(uint8_t gb_flg) {
    read_spi(ICM42688reg::GYRO_DATA_Z0);
    int16_t gyro_data_z0 = recvbuf[0]; 
    read_spi(ICM42688reg::GYRO_DATA_Z1);
    int16_t gyro_data_z1 = recvbuf[0]; 
    int16_t gyro_z_raw = (gyro_data_z1 << 8) | gyro_data_z0;  
    double gyro_z = gyro_z_raw * (gyro_fsr / 32768.0)  * (PI / 180); 
    if(gb_flg == 0) {
        gyro_z -= gyro_bias[2];
    } 
    // printf("Gyro Z: %f\n", gyro_z);
    return gyro_z;
}

int16_t icm42688_spi::get_ax0() {
    read_spi(ICM42688reg::ACCEL_DATA_X0);
    int16_t accel_data_x0 = recvbuf[0];
    return accel_data_x0;
}

int16_t icm42688_spi::get_ax1() {
    read_spi(ICM42688reg::ACCEL_DATA_X1);
    int16_t accel_data_x1 = recvbuf[0];
    return accel_data_x1;
}

int16_t icm42688_spi::get_ay0() {
    read_spi(ICM42688reg::ACCEL_DATA_Y0);
    int16_t accel_data_y0 = recvbuf[0];
    return accel_data_y0;
}

int16_t icm42688_spi::get_ay1() {
    read_spi(ICM42688reg::ACCEL_DATA_Y1);
    int16_t accel_data_y1 = recvbuf[0];
    return accel_data_y1;
}

int16_t icm42688_spi::get_az0() {
    read_spi(ICM42688reg::ACCEL_DATA_Z0);
    int16_t accel_data_z0 = recvbuf[0];
    return accel_data_z0;
}

int16_t icm42688_spi::get_az1() {
    read_spi(ICM42688reg::ACCEL_DATA_Z1);
    int16_t accel_data_z1 = recvbuf[0];
    return accel_data_z1;
}

int16_t icm42688_spi::get_gx0() {
    read_spi(ICM42688reg::GYRO_DATA_X0);
    int16_t gyro_data_x0 = recvbuf[0];
    return gyro_data_x0;
}

int16_t icm42688_spi::get_gx1() {
    read_spi(ICM42688reg::GYRO_DATA_X1);
    int16_t gyro_data_x1 = recvbuf[0];
    return gyro_data_x1;
}

int16_t icm42688_spi::get_gy0() {
    read_spi(ICM42688reg::GYRO_DATA_Y0);
    int16_t gyro_data_y0 = recvbuf[0];
    return gyro_data_y0;
}

int16_t icm42688_spi::get_gy1() {
    read_spi(ICM42688reg::GYRO_DATA_Y1);
    int16_t gyro_data_y1 = recvbuf[0];
    return gyro_data_y1;
}

int16_t icm42688_spi::get_gz0() {
    read_spi(ICM42688reg::GYRO_DATA_Z0);
    int16_t gyro_data_z0 = recvbuf[0];
    return gyro_data_z0;
}

int16_t icm42688_spi::get_gz1() {
    read_spi(ICM42688reg::GYRO_DATA_Z1);
    int16_t gyro_data_z1 = recvbuf[0];
    return gyro_data_z1;
}