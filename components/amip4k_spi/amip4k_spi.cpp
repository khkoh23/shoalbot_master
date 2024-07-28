#include "amip4k_spi.h"

amip4k_spi::amip4k_spi(amip4k_spi_config *spi_config) {
    busESP.mosi_io_num = spi_config->mosi;
    busESP.miso_io_num = spi_config->miso;
    busESP.sclk_io_num = spi_config->sclk;
    busESP.quadwp_io_num = -1;
    busESP.quadhd_io_num = -1; 

    IC_dev.command_bits = 0; 
    IC_dev.address_bits = 0; 
    IC_dev.dummy_bits = 0;
    IC_dev.clock_speed_hz = 1000000; // 1MHz
    IC_dev.duty_cycle_pos = 128;
    IC_dev.mode = 0;
    IC_dev.spics_io_num = spi_config->cs; 
    IC_dev.cs_ena_pretrans = 0.5;
    IC_dev.cs_ena_posttrans = 1; 
    IC_dev.queue_size = 5;

    HWA = spi_config -> hwa;
    bus_init = spi_config -> init_bus;
}

esp_err_t amip4k_spi::begin() {
    esp_err_t ret;
    if(!bus_init) {
        ret = spi_bus_initialize(SPI2_HOST, &busESP, SPI_DMA_CH_AUTO);
        if(ret != ESP_OK) return ret;
    }
    ret = spi_bus_add_device(SPI2_HOST, &IC_dev, &handle);
    if(ret != ESP_OK) return ret;
    // CHECK STAT ID
    ret = read_spi(AMIP4kreg::STAT_A, RD0);
    ret = read_spi(0x00, RD1);
    recvbuf[0] = SPI_SWAP_DATA_RX(recvbuf[0], 16);
    ret = read_spi(0x00, NOP);
    recvbuf[0] = SPI_SWAP_DATA_RX(recvbuf[0], 16);
    // assert(recvbuf[0] == 0x4300); // can change to WHILE LOOP
    ret = rate_conf(InterpolationRate::rate_4);
    if(ret != ESP_OK) return ret;
    ret = write_CFG3();
    return ret;
}


esp_err_t amip4k_spi::readSTAT() { 
    esp_err_t ret = read_spi(AMIP4kreg::STAT_A, RD0);
    // printf("Received data[0]: 0x%04X\n", recvbuf[0]);
    ret = read_spi(0x00, RD1);
    recvbuf[0] = SPI_SWAP_DATA_RX(recvbuf[0], 16);
    printf("Received data[1]: 0x%04X\n", recvbuf[0]);
    ret = read_spi(0x00, NOP);
    recvbuf[0] = SPI_SWAP_DATA_RX(recvbuf[0], 16);
    printf("Received data[2]: 0x%04X\n", recvbuf[0]);
    return ret;
}

int32_t amip4k_spi::readMVAL() { 
//    esp_err_t ret = read_spi(AM_IP_4kreg::MVAL_A, RD0);
    read_spi(AMIP4kreg::MVAL_A, RD0);
    // printf("Received data[0]: 0x%04X\n", recvbuf[0]);
//    ret = read_spi(0x00, RD1);
    read_spi(0x00, RD1);
    recvbuf[0] = SPI_SWAP_DATA_RX(recvbuf[0], 16);
    // printf("Received data[1]: 0x%04X\n", recvbuf[0]);
    MVAL = (MVAL & 0) | recvbuf[0];
//    ret = read_spi(0x00, NOP);
    read_spi(0x00, NOP);
    recvbuf[0] = SPI_SWAP_DATA_RX(recvbuf[0], 16);
    // printf("Received data[2]: 0x%04X\n", recvbuf[0]);
    MVAL |= (recvbuf[0] << 16);
//    printf("MVAL: 0x%08X\n", (unsigned int)MVAL);
//    uint8_t triggerValue = (MVAL >> 1) & 0x01;
//    bool isError = (MVAL & 0x01) != 0;
    MVAL = MVAL >> 2;
//    std::cout << "Measured Value: " << MVAL << std::endl;
//    std::cout << "Trigger Value: " << static_cast<int>(triggerValue) << std::endl;
//    std::cout << "Measured Value Error: " << (isError ? "Yes" : "No") << std::endl;

    return MVAL;
}

int32_t amip4k_spi::readCNT() {
    read_spi(AMIP4kreg::CNT_A, RD0);
    read_spi(0x00, RD1);
    recvbuf[0] = SPI_SWAP_DATA_RX(recvbuf[0], 16);
    CNT = (CNT & 0) | recvbuf[0];
    read_spi(0x00, NOP);
    recvbuf[0] = SPI_SWAP_DATA_RX(recvbuf[0], 16);
    CNT |= (recvbuf[0] << 16);
    CNT = CNT >> 2;
    return CNT;
}

int32_t amip4k_spi::readPOSIT() {
    read_spi(AMIP4kreg::POSIT_A, RD0);
    read_spi(0x00, RD1);
    recvbuf[0] = SPI_SWAP_DATA_RX(recvbuf[0], 16);
    POSIT = (POSIT & 0) | recvbuf[0];
    read_spi(0x00, NOP);
    recvbuf[0] = SPI_SWAP_DATA_RX(recvbuf[0], 16);
    POSIT |= (recvbuf[0] << 16);
    POSIT = POSIT >> 2;
    return POSIT;
}


esp_err_t amip4k_spi::rate_conf(InterpolationRate rate) {
    esp_err_t ret;
    if(rate == InterpolationRate::rate_16 || rate == InterpolationRate::rate_8 || rate == InterpolationRate::rate_4) {
        CFG1[0] = (CFG1[0] & 0b11100000) | InterpolationRate::rate_32;
        ret = write_CFG1();
        if(ret != ESP_OK) return ret;
        CFG2[0] = (CFG2[0] & 0b00011111) | (rate << 5);
        ret = write_CFG2();
        if(ret != ESP_OK) return ret;
    }
    else {
        CFG1[0] = (CFG1[0] & 0b11100000) | rate;
        ret = write_CFG1();
        if(ret != ESP_OK) return ret;
        CFG2[0] = (CFG2[0] & 0b00011111);
        ret = write_CFG2();
        if(ret != ESP_OK) return ret;
    }
    return ret;
}

esp_err_t amip4k_spi::reset_cnt() {
    esp_err_t ret = write_spi(AMIP4kreg::CMD_A, WRA);
    if(ret != ESP_OK) return ret;
    ret = write_spi(0x01, WRD);
    if(ret != ESP_OK) return ret;
    return ret;
}

esp_err_t amip4k_spi::write_CFG1() {
    esp_err_t ret = write_spi(AMIP4kreg::CFG1_A, WRA);
    if(ret != ESP_OK) return ret;
    ret = write_spi(CFG1[0], WRD);
    if(ret != ESP_OK) return ret;
    ret = write_spi(AMIP4kreg::CFG1_B, WRA);
    if(ret != ESP_OK) return ret;
    ret = write_spi(CFG1[1], WRD);
    if(ret != ESP_OK) return ret;
    ret = write_spi(AMIP4kreg::CFG1_C, WRA);
    if(ret != ESP_OK) return ret;
    ret = write_spi(CFG1[2], WRD);
    if(ret != ESP_OK) return ret;
    ret = write_spi(AMIP4kreg::CFG1_D, WRA);
    if(ret != ESP_OK) return ret;
    ret = write_spi(CFG1[3], WRD); 
    if(ret != ESP_OK) return ret;

    // printf("Writing to CFG1\n");

    ret = read_spi(AMIP4kreg::CFG1_A, RD0);
    recvbuf[0] = SPI_SWAP_DATA_RX(recvbuf[0], 16);
    // printf("Received data[0]: 0x%04X\n", recvbuf[0]);
    ret = read_spi(0x00, RD1);
    recvbuf[0] = SPI_SWAP_DATA_RX(recvbuf[0], 16);
    CFG1[0] = recvbuf[0] & 0x00FF;
    CFG1[1] = (recvbuf[0] & 0xFF00) >> 8;
    // printf("Received data[1]: 0x%04X\n", recvbuf[0]);
    ret = read_spi(0x00, NOP);
    recvbuf[0] = SPI_SWAP_DATA_RX(recvbuf[0], 16);
    CFG1[2] = recvbuf[0] & 0x00FF;
    CFG1[3] = (recvbuf[0] & 0xFF00) >> 8;
    // printf("Received data[2]: 0x%04X\n", recvbuf[0]);
    return ret;
}

esp_err_t amip4k_spi::write_CFG2() {
    esp_err_t ret = write_spi(AMIP4kreg::CFG2_A, WRA);
    if(ret != ESP_OK) return ret;
    ret = write_spi(CFG2[0], WRD);
    if(ret != ESP_OK) return ret;
    ret = write_spi(AMIP4kreg::CFG2_B, WRA);
    if(ret != ESP_OK) return ret;
    ret = write_spi(CFG2[1], WRD);
    if(ret != ESP_OK) return ret;
    ret = write_spi(AMIP4kreg::CFG2_C, WRA);
    if(ret != ESP_OK) return ret;
    ret = write_spi(CFG2[2], WRD);
    if(ret != ESP_OK) return ret;
    ret = write_spi(AMIP4kreg::CFG2_D, WRA);
    if(ret != ESP_OK) return ret;
    ret = write_spi(CFG2[3], WRD); 
    if(ret != ESP_OK) return ret;

    // printf("Writing to CFG2\n");

    ret = read_spi(AMIP4kreg::CFG2_A, RD0);
    recvbuf[0] = SPI_SWAP_DATA_RX(recvbuf[0], 16);
    // printf("Received data[0]: 0x%04X\n", recvbuf[0]);
    ret = read_spi(0x00, RD1);
    recvbuf[0] = SPI_SWAP_DATA_RX(recvbuf[0], 16);
    CFG2[0] = recvbuf[0] & 0x00FF;
    CFG2[1] = (recvbuf[0] & 0xFF00) >> 8;
    // printf("Received data[1]: 0x%04X\n", recvbuf[0]);
    ret = read_spi(0x00, NOP);
    recvbuf[0] = SPI_SWAP_DATA_RX(recvbuf[0], 16);
    CFG2[2] = recvbuf[0] & 0x00FF;
    CFG2[3] = (recvbuf[0] & 0xFF00) >> 8;
    // printf("Received data[2]: 0x%04X\n", recvbuf[0]);
    return ret;
}

esp_err_t amip4k_spi::write_CFG3(bool abSwitch) {
    if(abSwitch) {
        CFG3[1] &= (0b1 <<5);
    }
    esp_err_t ret = write_spi(AMIP4kreg::CFG3_A, WRA);
    if(ret != ESP_OK) return ret;
    ret = write_spi(CFG3[0], WRD);
    if(ret != ESP_OK) return ret;
    ret = write_spi(AMIP4kreg::CFG3_B, WRA);
    if(ret != ESP_OK) return ret;
    ret = write_spi(CFG3[1], WRD);
    if(ret != ESP_OK) return ret;
    ret = write_spi(AMIP4kreg::CFG3_C, WRA);
    if(ret != ESP_OK) return ret;
    ret = write_spi(CFG3[2], WRD);
    if(ret != ESP_OK) return ret;
    ret = write_spi(AMIP4kreg::CFG3_D, WRA);
    if(ret != ESP_OK) return ret;
    ret = write_spi(CFG3[3], WRD); 
    if(ret != ESP_OK) return ret;
    
    // printf("Writing to CFG3\n");

    ret = read_spi(AMIP4kreg::CFG3_A, RD0);
    recvbuf[0] = SPI_SWAP_DATA_RX(recvbuf[0], 16);
    // printf("Received data[0]: 0x%04X\n", recvbuf[0]);
    ret = read_spi(0x00, RD1);
    recvbuf[0] = SPI_SWAP_DATA_RX(recvbuf[0], 16);
    CFG3[0] = recvbuf[0] & 0x00FF;
    CFG3[1] = (recvbuf[0] & 0xFF00) >> 8;
    // printf("Received data[1]: 0x%04X\n", recvbuf[0]);
    ret = read_spi(0x00, NOP);
    recvbuf[0] = SPI_SWAP_DATA_RX(recvbuf[0], 16);
    CFG3[2] = recvbuf[0] & 0x00FF;
    CFG3[3] = (recvbuf[0] & 0xFF00) >> 8;
    // printf("Received data[2]: 0x%04X\n", recvbuf[0]);
    return ret;
}

esp_err_t amip4k_spi::read_spi(uint8_t reg, uint8_t op_code) {
    // EACH TIME THE CLOCK CYCLE SHOULD BE ONLY 16 CYCLES
    t.length = 16;
    sendbuf[0] = (op_code << 12) | (HWA << 8) | reg;
    sendbuf[0] = SPI_SWAP_DATA_TX(sendbuf[0], 16);
    t.tx_buffer = sendbuf;
    t.rx_buffer = recvbuf;
    esp_err_t ret = spi_device_transmit(handle, &t);
    return ret;
}

esp_err_t amip4k_spi::write_spi(uint8_t reg, uint8_t op_code) {
    t.length = 16;
    sendbuf[0] = (op_code << 12) | (HWA << 8) | reg;
    sendbuf[0] = SPI_SWAP_DATA_TX(sendbuf[0], 16);
    t.tx_buffer = sendbuf;
    t.rx_buffer = 0;
    esp_err_t ret = spi_device_transmit(handle, &t);
    return ret;
}