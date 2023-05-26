#ifndef _DEFINES_H
#define _DEFINES_H

typedef sc_dt::sc_fixed<16, 8> data_t;
typedef sc_dt::sc_fixed<16, 9> angle_t;
typedef struct CORDIC_output {
    data_t x;
    data_t y;
    angle_t theta;
} CORDIC_output_t;

// PE inner transport addresses
const int PE_INPUT_A_ADDR = 0x00000000;
const int PE_INPUT_B_ADDR = 0x00000004;
const int PE_INPUT_Z_ADDR = 0x00000008;
const int PE_OUTPUT_A_ADDR = 0x0000000c;
const int PE_OUTPUT_B_ADDR = 0x00000010;
const int PE_OUTPUT_Z_ADDR = 0x00000014;

union word {
  float f;
  unsigned char uc[4];
};



#endif
