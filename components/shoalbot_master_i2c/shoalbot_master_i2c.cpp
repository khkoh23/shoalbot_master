#include "shoalbot_master_i2c.h"

shoalbot_master_i2c::shoalbot_master_i2c(i2c_master_config* conf) {
    i2c_mst_config.sda_io_num = conf -> sda;
    i2c_mst_config.scl_io_num = conf -> scl;
    i2c_mst_config.clk_source = I2C_CLK_SRC_DEFAULT;
    i2c_mst_config.i2c_port = I2C_NUM_0;
    i2c_mst_config.glitch_ignore_cnt = 7;
    i2c_mst_config.flags.enable_internal_pullup = true;
    i2c_mst_config.trans_queue_depth = 10;
//    i2c_mst_config.intr_priority = 1;

    ESP_slave.dev_addr_length = I2C_ADDR_BIT_LEN_7;
    ESP_slave.device_address = conf -> slaveAddr;
    ESP_slave.scl_speed_hz = 100000; 

    gpio_set_pull_mode(conf->sda, GPIO_PULLUP_ONLY);
    gpio_set_pull_mode(conf->scl, GPIO_PULLUP_ONLY);
}

esp_err_t shoalbot_master_i2c::begin() {
    ret = i2c_new_master_bus(&i2c_mst_config, &i2c_mst_handle);
    if(ret != ESP_OK) {
        //ESP_LOGE(TAG, "Failed to create I2C master bus");
        return ret;
    }
    ret = i2c_master_bus_add_device(i2c_mst_handle, &ESP_slave, &i2c_master_handle); 
    if(ret != ESP_OK) {
        //ESP_LOGE(TAG, "Failed to add slave device");
        return ret;
    }
    return ret;
}

esp_err_t shoalbot_master_i2c::i2c_send_DO(uint8_t* data) { // first byte 0xBB for DO cmd
    ret = i2c_master_transmit(i2c_master_handle, data, 1, 10);
    if(ret != ESP_OK) {
        //ESP_LOGE(TAG, "Failed to transmit data");
        return ret;
    }
    vTaskDelay(1);
    ret = i2c_master_transmit(i2c_master_handle, data + 1, 1, 10);
    if(ret != ESP_OK) {
        //ESP_LOGE(TAG, "Failed to transmit data");
        return ret;
    }
    vTaskDelay(1);
    ret = i2c_master_transmit(i2c_master_handle, data + 2, 1, 10);
    if(ret != ESP_OK) {
        //ESP_LOGE(TAG, "Failed to transmit data");
        return ret;
    }
    vTaskDelay(1);
    ret = i2c_master_transmit(i2c_master_handle, data + 3, 1, 10);
    if(ret != ESP_OK) {
        //ESP_LOGE(TAG, "Failed to transmit data");
        return ret;
    }
    vTaskDelay(1);
    ret = i2c_master_transmit(i2c_master_handle, data + 4, 1, 10);
    if(ret != ESP_OK) {
        //ESP_LOGE(TAG, "Failed to transmit data");
        return ret;
    }
    vTaskDelay(1);
    ret = i2c_master_transmit(i2c_master_handle, data + 5, 1, 10);
    if(ret != ESP_OK) {
        //ESP_LOGE(TAG, "Failed to transmit data");
        return ret;
    }
    // printf("sending DO\n");
    return ret;
}

uint32_t shoalbot_master_i2c::read_state() {
    DI_fromSlave = 0;
    ret = i2c_master_transmit_receive(i2c_master_handle, DI_cmd, sizeof(DI_cmd), state_data, 6, -1);
    if(ret != ESP_OK) {
        //ESP_LOGE(TAG, "Failed to rectrieve DI");
        return ret;
    }
    // ESP_LOGI("I2C", "Data Read 1: 0x%02X\n", state_data[0]);
    // ESP_LOGI("I2C", "Data Read 2: 0x%02X\n", state_data[1]);
    // ESP_LOGI("I2C", "Data Read 3: 0x%02X\n", state_data[2]);
    // ESP_LOGI("I2C", "Data Read 4: 0x%02X\n", state_data[3]);
    // ESP_LOGI("I2C", "Data Read 5: 0x%02X\n", state_data[4]);
    // ESP_LOGI("I2C", "Data Read 6: 0x%02X\n", state_data[5]);
    DI_fromSlave = ((uint32_t)state_data[0]) << 16 | ((uint32_t)state_data[1]) << 8 | ((uint32_t)state_data[2]);
    DO_slave[0] = state_data[3];
    DO_slave[1] = 0x03 & state_data[4];
    
    AMR_state[1] = state_data[4] >> 2;
    AMR_state[0] = state_data[5];

    ESP_LOGI("I2C", "DI: 0x%08X\n", static_cast<unsigned int>(DI_fromSlave));
    ESP_LOGI("I2C", "DO: 0x%02X\n", DO_slave[0]);
    ESP_LOGI("I2C", "DO1: 0x%02X\n", DO_slave[1]);
    ESP_LOGI("I2C", "AMR: 0x%02X\n", AMR_state[0]);
    ESP_LOGI("I2C", "AMR1: 0x%02X\n", AMR_state[1]);
    return DI_fromSlave;
}

// esp_err_t shoalbot_master_i2c::write_BMS(uint32_t data) { 
//     // Bit 0: BMS, Bit 1: Pass1, Bit 2: Pass2
//     uint8_t percentage[5] = {};
//     percentage[0] = 0xCC;
//     percentage[1] = (data >> 24) & 0xFF; // highest byte (MSB)
//     percentage[2] = (data >> 16) & 0xFF; // second highest byte
//     percentage[3] = (data >> 8) & 0xFF;  // second lowest byte
//     percentage[4] = data & 0xFF;         // lowest byte (LSB)
//     ret = i2c_master_transmit(i2c_master_handle, percentage, 1, 10);
//         if(ret != ESP_OK) {
//             ESP_LOGE(TAG, "Failed to transmit data");
//             return ret;
//         }
//     vTaskDelay(1);
//     ret = i2c_master_transmit(i2c_master_handle, percentage + 1, 1, 10);
//         if(ret != ESP_OK) {
//             ESP_LOGE(TAG, "Failed to transmit data");
//             return ret;
//         }
//     vTaskDelay(1);
//     ret = i2c_master_transmit(i2c_master_handle, percentage + 2, 1, 10);
//         if(ret != ESP_OK) {
//             ESP_LOGE(TAG, "Failed to transmit data");
//             return ret;
//         }
//     vTaskDelay(1);
//     ret = i2c_master_transmit(i2c_master_handle, percentage + 3, 1, 10);
//         if(ret != ESP_OK) {
//             ESP_LOGE(TAG, "Failed to transmit data");
//             return ret;
//         }
//     vTaskDelay(1);
//     ret = i2c_master_transmit(i2c_master_handle, percentage + 4, 1, 10);
//         if(ret != ESP_OK) {
//             ESP_LOGE(TAG, "Failed to transmit data");
//             return ret;
//         }
//     return ret;
// }
/*
bool shoalbot_master_i2c::check_BATSW() {
    ret = i2c_master_transmit_receive(i2c_master_handle, batSW_cmd, sizeof(batSW_cmd), &batSW, 1, -1);
    //printf("BATSW: %d\n", batSW);
    if(ret != ESP_OK) {
        //ESP_LOGE(TAG, "Failed to rectrieve BATSW");
        return ret;
    }
    if(batSW == 0x01) {
        printf("Battery Switch is ON\n");
        return true; // true meaning switched on
    }
    else {
        printf("Battery Switch is OFF\n");
        return false;
    }
}*/