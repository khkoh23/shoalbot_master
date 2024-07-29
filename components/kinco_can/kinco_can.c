#include "kinco_can.h"

void initTwai (const uint8_t tx, const uint8_t rx) {
    twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(tx, rx, TWAI_MODE_NORMAL);
    //twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(tx, rx, TWAI_MODE_NO_ACK);
    //twai_general_config_t g_config = {.mode = TWAI_MODE_NORMAL, .tx_io = tx, .rx_io = rx, .tx_queue_len = 0};
//    g_config.intr_flags = ESP_INTR_FLAG_HIGH;
    twai_timing_config_t t_config = TWAI_TIMING_CONFIG_500KBITS();
    twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();
    if (twai_driver_install(&g_config, &t_config, &f_config) == ESP_OK) {
        //printf("Driver installed\n");
    } else {
        //printf("Failed to install driver\n");
        return;
    }
    if (twai_start() == ESP_OK) {
        //printf("Driver started\n");
    } else {
        //printf("Failed to start driver\n");
        return;
    }
    setModesOfOperation(1, 3); 
    setModesOfOperation(2, 3); 
}

void setTargetVelocity (const uint8_t id, const int32_t msg) { //60FF 
    int32_t rpm = msg*2730.6666667*-1; //TODO why?
    uint8_t d1 = (rpm & 0x000000FF) >> 0; 
    uint8_t d2 = (rpm & 0x0000FF00) >> 8; 
    uint8_t d3 = (rpm & 0x00FF0000) >> 16; 
    uint8_t d4 = (rpm & 0xFF000000) >> 24; 
    twai_message_t message = {.ss = 1, .identifier = 0x600 + id, .data_length_code = 8, .data = {0x23, 0xFF, 0x60, 0x00, d1, d2, d3, d4} };
    if (twai_transmit(&message, pdMS_TO_TICKS(1000)) == ESP_OK) {
        //printf("Message queued for transmission\n");
    }
    else {
        //printf("Failed to queue message for transmission\n");
    }
}

void setModesOfOperation (const uint8_t id, const int8_t msg) { //6060 
    uint8_t d1 = (msg & 0xFF) >> 0; 
    twai_message_t message = {.identifier = 0x600 + id, .data_length_code = 8, .data = {0x2F, 0x60, 0x60, 0x00, d1, 0x00, 0x00, 0x00} };
    if (twai_transmit(&message, pdMS_TO_TICKS(1000)) == ESP_OK) {
        //printf("Message queued for transmission\n");
    }
    else {
        //printf("Failed to queue message for transmission\n");
    } 
}

void setProfileAcceleration (const uint8_t id, const uint32_t msg) { //6083
    uint32_t rpsps = msg*1; //TODO
    uint8_t d1 = (rpsps & 0x000000FF) >> 0; 
    uint8_t d2 = (rpsps & 0x0000FF00) >> 8; 
    uint8_t d3 = (rpsps & 0x00FF0000) >> 16; 
    uint8_t d4 = (rpsps & 0xFF000000) >> 24; 
    twai_message_t message = {.identifier = 0x600 + id, .data_length_code = 8, .data = {0x23, 0x83, 0x60, 0x00, d1, d2, d3, d4} };
    if (twai_transmit(&message, pdMS_TO_TICKS(1000)) == ESP_OK) {
        //printf("Message queued for transmission\n");
    }
    else {
        //printf("Failed to queue message for transmission\n");
    }
}

void setProfileDeceleration (const uint8_t id, const uint32_t msg) { //6084
    uint32_t rpsps = msg*1; //TODO
    uint8_t d1 = (rpsps & 0x000000FF) >> 0; 
    uint8_t d2 = (rpsps & 0x0000FF00) >> 8; 
    uint8_t d3 = (rpsps & 0x00FF0000) >> 16; 
    uint8_t d4 = (rpsps & 0xFF000000) >> 24; 
    twai_message_t message = {.identifier = 0x600 + id, .data_length_code = 8, .data = {0x23, 0x84, 0x60, 0x00, d1, d2, d3, d4} };
    if (twai_transmit(&message, pdMS_TO_TICKS(1000)) == ESP_OK) {
        //printf("Message queued for transmission\n");
    }
    else {
        //printf("Failed to queue message for transmission\n");
    }
}


/*
    //Wait for message to be received
//  twai_message_t message;
    if (twai_receive(&message, pdMS_TO_TICKS(10000)) == ESP_OK) {
        printf("Message received\n");
    } else {
        printf("Failed to receive message\n");
        return;
    }
    //Process received message
    if (message.extd) {
        printf("Message is in Extended Format\n");
    } else {
        printf("Message is in Standard Format\n");
    }
    printf("ID is %ld\n", message.identifier);
    if (!(message.rtr)) {
        for (int i = 0; i < message.data_length_code; i++) {
            printf("Data byte %d = %d\n", i, message.data[i]);
        }
    }
*/

/*
    //Stop the TWAI driver
    if (twai_stop() == ESP_OK) {
        printf("Driver stopped\n");
    } else {
        printf("Failed to stop driver\n");
        return;
    }
    //Uninstall the TWAI driver
    if (twai_driver_uninstall() == ESP_OK) {
        printf("Driver uninstalled\n");
    } else {
        printf("Failed to uninstall driver\n");
        return;
    }
*/