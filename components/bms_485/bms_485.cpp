/**
 * This file contains the implementation of the bms_485 class.
 *
 * The bms_485 class is used to communicate with the BMS system in half duplex mode.
 * It initializes the UART communication, also contains the implementation of methods 
 * to get the BMS data, get voltage, get current, get charge, get capacity, etc.
 * 
 * @file bms_485.cpp
 */

#include <stdio.h>
#include "bms_485.h"

/**
 * Initializes the bms_485 object by setting up the UART communication.
 * 
 * This function sets up the UART driver, configures the UART parameters, 
 * sets the UART pins, sets the RS485 half duplex mode, and sets the read 
 * timeout of the UART TOUT feature. It also sends a command to the BMS 
 * system to initialize the communication.
 * 
 * @return None.
 */
bms_485::bms_485(){
    bms_uart_num = BMS_UART_PORT;
    uart_config_t uart_config = {
        .baud_rate = BMS_BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .rx_flow_ctrl_thresh = 122,
        .source_clk = UART_SCLK_APB,
    };

    // Set UART log level
    esp_log_level_set(BMS_TAG, ESP_LOG_INFO);

    ESP_LOGI(BMS_TAG, "Start RS485 application test and configure UART.");

    // Install UART driver (we don't need an event queue here)
    ESP_ERROR_CHECK(uart_driver_install(bms_uart_num, BMS_BUF_SIZE * 2, 0, 0, NULL, 0));
        
    // Configure UART parameters
    ESP_ERROR_CHECK(uart_param_config(bms_uart_num, &uart_config));

    // Set UART pins as per KConfig settings
    ESP_ERROR_CHECK(uart_set_pin(bms_uart_num, BMS_TEST_TXD, BMS_TEST_RXD, BMS_TEST_RTS, BMS_TEST_CTS));

    // Set RS485 half duplex mode
    ESP_ERROR_CHECK(uart_set_mode(bms_uart_num, UART_MODE_RS485_HALF_DUPLEX));

    // Set read timeout of UART TOUT feature
    ESP_ERROR_CHECK(uart_set_rx_timeout(bms_uart_num, ECHO_READ_TOUT));

    bms_echo_send(bms_uart_num, "\r\n", 2);
}

/**
 * Retrieves BMS data from the UART and decodes various values based on the received data.
 *
 * @throws None
 */
void bms_485::getBMSData(){
    // Allocate buffers for UART
    uint8_t data[BMS_BUF_SIZE];
    int len = 0;  // Number of bytes received
    uint8_t buf = 0;
    while (len < 10){
        // Write data over UART to BMS.
        bms_echo_send(bms_uart_num, "\r\n", 2);
        vTaskDelay(10/portTICK_PERIOD_MS);
        //Read data from UART
        len = uart_read_bytes(bms_uart_num, data, BMS_BUF_SIZE, PACKET_READ_TICS);
        // The header of the pakcet will be 0x7E, 0X32.
        // So check the header first.
        while (data[buf] != 126 || data[buf+1] != 50) buf++;
        // Ignore some data in front.  
        buf += (uint8_t)BUFFER_BIAS;
        /* CALCULATE BY BYTES START*/
        // Temperature should minus 40 to convert to degree, according to the datasheet.
        _cell_temperature[0] = (hex_to_dec(data[buf]) << 12) + (hex_to_dec(data[buf + 1]) << 8)
                    + (hex_to_dec(data[buf + 2]) << 4) + (hex_to_dec(data[buf + 3])) - 40;
        _cell_temperature[1] = (hex_to_dec(data[buf + 4]) << 12) + (hex_to_dec(data[buf + 5]) << 8)
                    + (hex_to_dec(data[buf + 6]) << 4) + (hex_to_dec(data[buf + 7])) - 40;
        _cell_temperature[2] = (hex_to_dec(data[buf + 8]) << 12) + (hex_to_dec(data[buf + 9]) << 8)
                            + (hex_to_dec(data[buf + 10]) << 4) + (hex_to_dec(data[buf + 11])) - 40;
        // Cast to int16_t to Calculate current, which can be negative, unit is 10 mA.
        _current = (int16_t)((hex_to_dec(data[buf + 12]) << 12) + (hex_to_dec(data[buf + 13]) << 8)
                            + (hex_to_dec(data[buf + 14]) << 4) + hex_to_dec(data[buf + 15])) * 10;
        // Cast to int16_t to Calculate voltage, which can be negative, unit is 1 mV.
        _voltage = (hex_to_dec(data[buf + 16]) << 12) + (hex_to_dec(data[buf + 17]) << 8)
                            + (hex_to_dec(data[buf + 18]) << 4) + (hex_to_dec(data[buf + 19]));
        _charge = (hex_to_dec(data[buf + 20]) << 12) + (hex_to_dec(data[buf + 21]) << 8)
                            + (hex_to_dec(data[buf + 22]) << 4) + (hex_to_dec(data[buf + 23]));
        _capacity = (hex_to_dec(data[buf + 26]) << 12) + (hex_to_dec(data[buf + 27]) << 8)
                            + (hex_to_dec(data[buf + 28]) << 4) + (hex_to_dec(data[buf + 29]));
        // _cycle = (hex_to_dec(data[buf + 18]) << 12) + (hex_to_dec(data[buf + 19]) << 8)
        //                     + (hex_to_dec(data[buf + 20]) << 4) + (hex_to_dec(data[buf + 21]));
        /* CALCULATE BY BYTES END*/
    }
}

/**
 * Sends data over UART and handles sending failure.
 *
 * @param port the UART port number
 * @param str pointer to the data buffer to be sent
 * @param length the length of the data to be sent
 *
 * @throws None
 */
void bms_485::bms_echo_send(uart_port_t port, const char* str, uint8_t length)
{
    if (uart_write_bytes(port, str, length) != length) {
        ESP_LOGE(BMS_TAG, "Send data critical failure.");
        // add your code to handle sending failure here
        abort();
    }
}

/**
 * Converts a hexadecimal value to its decimal equivalent.
 *
 * @param val the hexadecimal value to be converted
 *
 * @return the decimal equivalent of the input value
 *
 * @throws None
 */
uint8_t bms_485::hex_to_dec(uint8_t val) {
    if(val >= '0' && val <= '9')
    {
        val = val - 48;
    }
    else if(val >= 'A' && val <= 'F')
    {
        val = val - 65 + 10;
    }
    return val;
}

float_t bms_485::getTemperature() {
    _temperature = (_cell_temperature[0] + _cell_temperature[1] + _cell_temperature[2]) / 3.0;
    return _temperature;
}

float_t bms_485::getVoltage() {
    return _voltage;
}

float_t bms_485::getCurrent() {
    return _current;
}

float_t bms_485::getCharge(){
    return _charge;
}

float_t bms_485::getCapacity() {
    return _capacity;
}

float_t bms_485::getDesignCapacity() {
    return _design_capacity;
}

float_t bms_485::getPercentage() {
    return _percentage;
}

float_t* bms_485::getCellTemperature() {
    return _cell_temperature;
}