#ifndef AMIP4k_REGISTER_H
#define AMIP4k_REGISTER_H

#include <stdint.h>

namespace AMIP4kreg {
    static constexpr uint8_t MVAL_A = 0x00;
    static constexpr uint8_t MVAL_B = 0x01;
    static constexpr uint8_t MVAL_C = 0x02;
    static constexpr uint8_t MVAL_D = 0x03;

    static constexpr uint8_t CNT_A = 0x04;
    static constexpr uint8_t CNT_B = 0x05;
    static constexpr uint8_t CNT_C = 0x06;
    static constexpr uint8_t CNT_D = 0x07;

    static constexpr uint8_t STAT_A = 0x08;
    static constexpr uint8_t STAT_B = 0x09;
    static constexpr uint8_t STAT_C = 0x0A;
    static constexpr uint8_t STAT_D = 0x0B;

    static constexpr uint8_t CFG1_A = 0x0C; // 0b00001100
    static constexpr uint8_t CFG1_B = 0x0D;
    static constexpr uint8_t CFG1_C = 0x0E;
    static constexpr uint8_t CFG1_D = 0x0F;

    static constexpr uint8_t CFG2_A = 0x10;
    static constexpr uint8_t CFG2_B = 0x11;
    static constexpr uint8_t CFG2_C = 0x12;
    static constexpr uint8_t CFG2_D = 0x13;

    static constexpr uint8_t CFG3_A = 0x14;
    static constexpr uint8_t CFG3_B = 0x15;
    static constexpr uint8_t CFG3_C = 0x16;
    static constexpr uint8_t CFG3_D = 0x17;

    static constexpr uint8_t CFG4_A = 0x18;
    static constexpr uint8_t CFG4_B = 0x19;
    static constexpr uint8_t CFG4_C = 0x1A;
    static constexpr uint8_t CFG4_D = 0x1B;

    static constexpr uint8_t CNTRLG_A = 0x1C;
    static constexpr uint8_t CNTRLG_B = 0x1D;
    static constexpr uint8_t CNTRLG_C = 0x1E;
    static constexpr uint8_t CNTRLG_D = 0x1F;

    static constexpr uint8_t CNTRLO_A = 0x20;
    static constexpr uint8_t CNTRLO_B = 0x21;
    static constexpr uint8_t CNTRLO_C = 0x22;
    static constexpr uint8_t CNTRLO_D = 0x23;

    static constexpr uint8_t PRE_ST_A = 0x24;
    static constexpr uint8_t PRE_ST_B = 0x25;
    static constexpr uint8_t PRE_ST_C = 0x26;
    static constexpr uint8_t PRE_ST_D = 0x27;

    static constexpr uint8_t PRE_MT_A = 0x28;
    static constexpr uint8_t PRE_MT_B = 0x29;
    static constexpr uint8_t PRE_MT_C = 0x2A;
    static constexpr uint8_t PRE_MT_D = 0x2B;

    static constexpr uint8_t CFGIUW_A = 0x2E;
    static constexpr uint8_t CFGIUW_B = 0x2F;
    
    static constexpr uint8_t CFGSSI_A = 0x30;
    static constexpr uint8_t CFGSSI_B = 0x31;
    static constexpr uint8_t CFGSSI_C = 0x32;
    static constexpr uint8_t CFGSSI_D = 0x33;

    static constexpr uint8_t CFGLDR_A = 0x34;
    static constexpr uint8_t CFGLDR_B = 0x35;
    static constexpr uint8_t CFGLDR_C = 0x36;
    static constexpr uint8_t CFGLDR_D = 0x37;

    static constexpr uint8_t CFGLDR2_A = 0x38;
    static constexpr uint8_t CFGLDR2_B = 0x39;
    static constexpr uint8_t CFGLDR2_C = 0x3A;
    static constexpr uint8_t CFGLDR2_D = 0x3B;

    static constexpr uint8_t EEP_DAT_A = 0x48;
    static constexpr uint8_t EEP_DAT_B = 0x49;

    static constexpr uint8_t EEP_ADR = 0x4A;
    static constexpr uint8_t EEP_OPC = 0x4B;

    static constexpr uint8_t CFGTM_A = 0x4C;
    static constexpr uint8_t CFGTM_B = 0x4D;
    static constexpr uint8_t CFGTM_C = 0x4E;
    static constexpr uint8_t CFGTM_D = 0x4F;

    static constexpr uint8_t CMD_A = 0x50;
    static constexpr uint8_t CMD_B = 0x51;

    static constexpr uint8_t TSTCMD_A = 0x52;
    static constexpr uint8_t TSTCMD_B = 0x53;

    static constexpr uint8_t CFGEEP_A = 0x54;
    static constexpr uint8_t CFGEEP_B = 0x55;
    static constexpr uint8_t CFGEEP_C = 0x56;
    static constexpr uint8_t CFGEEP_D = 0x57;

    static constexpr uint8_t POSIT_A = 0x80;
    static constexpr uint8_t POSIT_B = 0x81;
    static constexpr uint8_t POSIT_C = 0x82;
    static constexpr uint8_t POSIT_D = 0x83;

    static constexpr uint8_t ADC_A = 0x84;
    static constexpr uint8_t ADC_B = 0x85;
    static constexpr uint8_t ADC_C = 0x86;
    static constexpr uint8_t ADC_D = 0x87;

    static constexpr uint8_t CADC_A = 0x88;
    static constexpr uint8_t CADC_B = 0x89;
    static constexpr uint8_t CADC_C = 0x8A;
    static constexpr uint8_t CADC_D = 0x8B;

    static constexpr uint8_t IP1_A = 0x8C;
    static constexpr uint8_t IP1_B = 0x8D;
    static constexpr uint8_t IP1_C = 0x8E;
    static constexpr uint8_t IP1_D = 0x8F;

    static constexpr uint8_t IP2_A = 0x90;
    static constexpr uint8_t IP2_B = 0x91;
    static constexpr uint8_t IP2_C = 0x92;
    static constexpr uint8_t IP2_D = 0x93;

    static constexpr uint8_t Korrekturwert_SC_A = 0x94; //Correction value
    static constexpr uint8_t Korrekturwert_SC_B = 0x95;
    static constexpr uint8_t Korrekturwert_SC_C = 0x96;
    static constexpr uint8_t Korrekturwert_SC_D = 0x97;

    static constexpr uint8_t Korrekturwert_360_A = 0x98;
    static constexpr uint8_t Korrekturwert_360_B = 0x99;
    static constexpr uint8_t Korrekturwert_360_C = 0x9A;
    static constexpr uint8_t Korrekturwert_360_D = 0x9B;

    static constexpr uint8_t LDR_OUT_A = 0x9C;
    static constexpr uint8_t LDR_OUT_B = 0x9D;
    static constexpr uint8_t LDR_OUT_C = 0x9E;
    static constexpr uint8_t LDR_OUT_D = 0x9F;
}


#endif // AM_IP_4k_REGISTER_H