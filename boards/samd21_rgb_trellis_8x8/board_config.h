#ifndef SEESAW_DEVICE_CONFIG_H
#define SEESAW_DEVICE_CONFIG_H

#define PRODUCT_CODE 0000

//override default I2C addr
#define CONFIG_I2C_SLAVE_ADDR 0x2E

#define CONFIG_NO_ADDR 1
#define CONFIG_NO_ACTIVITY_LED
#define CONFIG_NO_EEPROM

// #define CONFIG_ADDR_2 1
// #define CONFIG_ADDR_3 1
// #define CONFIG_ADDR_4 1
// #define PIN_ADDR_0 17
// #define PIN_ADDR_1 16
// #define PIN_ADDR_2 15
// #define PIN_ADDR_3 14
// #define PIN_ADDR_4 11

//* ============== POOL SIZES =================== *//
#define	EVT_SIZE_SMALL 16
#define EVT_SIZE_MEDIUM 32
#define	EVT_SIZE_LARGE 64
#define	EVT_COUNT_SMALL 16
#define	EVT_COUNT_MEDIUM 8
#define	EVT_COUNT_LARGE 1

//* ============== ADC =================== *//
#define CONFIG_ADC 0
#define CONFIG_ADC_INPUT_0 1

#define CONFIG_ADC_INPUT_1 1

#define CONFIG_ADC_INPUT_2 0

#define CONFIG_ADC_INPUT_3 0

//* ============== DAC =================== *//
#define CONFIG_DAC 0

//* ============== TIMER =================== *//
#define CONFIG_TIMER 0

//override default PWM0 pin
#define CONFIG_TIMER_PWM_OUT0 1

#define CONFIG_TIMER_PWM_OUT0_TC TC1
#define CONFIG_TIMER_PWM_OUT0_WO 1
#define CONFIG_TIMER_PWM_OUT0_PIN 5

#define CONFIG_TIMER_PWM_OUT1 0

#define CONFIG_TIMER_PWM_OUT2 0

#define CONFIG_TIMER_PWM_OUT3 0

//* ============== INTERRUPT =================== *//
#define CONFIG_INTERRUPT 1
#define CONFIG_INTERRUPT_PIN 27

#define CONFIG_INTERRUPT_OPEN_DRAIN 1

//* ============== I2C SLAVE =================== *//
#define CONFIG_I2C_SLAVE 1

//override default mux
#define CONFIG_I2C_SLAVE_MUX 2

//#define CONFIG_I2C_SLAVE_FLOW_CONTROL 1

//override the default pins
#define CONFIG_I2C_SLAVE_PIN_SDA 22
#define CONFIG_I2C_SLAVE_PIN_SCL 23


#define CONFIG_I2C_SLAVE_SERCOM SERCOM3
#define CONFIG_I2C_SLAVE_HANDLER SERCOM3_Handler
#define CONFIG_I2C_SLAVE_IRQn SERCOM3_IRQn




//* ============== SERCOM =================== *//
#define CONFIG_SERCOM0 0
#define CONFIG_SERCOM1 0
#define CONFIG_SERCOM2 0

//These are only available on samd21
#define CONFIG_SERCOM3 0
#define CONFIG_SERCOM4 0
#define CONFIG_SERCOM5 0

//* ============== DAP =================== *//
#define CONFIG_DAP 0

#define CONFIG_DAP_SWCLK 30
#define CONFIG_DAP_SWDIO 31
#define CONFIG_DAP_TDI 19
#define CONFIG_DAP_TDO 20
#define CONFIG_DAP_nTRST 21 /*??*/
#define CONFIG_DAP_nRESET 22 /*??*/ 

//* =========== NEOPIXEL ================ *//
#define CONFIG_NEOPIXEL 1

#define CONFIG_NEOPIXEL_BUF_MAX (64*4)

//* =========== BLINK ================ *//
#define CONFIG_BLINK 0
#define PIN_ACTIVITY_LED 17

//* ============== ENCODER =================== *//
#define CONFIG_ENCODER 0
#define CONFIG_NUM_ENCODERS 1

#define CONFIG_ENCODER0_A_PIN 2
#define CONFIG_ENCODER0_B_PIN 4

// #define CONFIG_ENCODER1_A_PIN 9
// #define CONFIG_ENCODER1_B_PIN 4
//
// #define CONFIG_ENCODER2_A_PIN 5
// #define CONFIG_ENCODER2_B_PIN 2
//
// #define CONFIG_ENCODER3_A_PIN 11
// #define CONFIG_ENCODER3_B_PIN 10
//
#define CONFIG_ENCODER_HANDLER TC3_Handler
#define CONFIG_ENCODER_IRQn TC3_IRQn
#define CONFIG_ENCODER_TC TC3

//* =========== KEYPAD ================ *//
#define CONFIG_KEYPAD 1

#define CONFIG_KEYPAD_ROW0 1
#define CONFIG_KEYPAD_ROW1 1
#define CONFIG_KEYPAD_ROW2 1
#define CONFIG_KEYPAD_ROW3 1
#define CONFIG_KEYPAD_ROW4 1
#define CONFIG_KEYPAD_ROW5 1
#define CONFIG_KEYPAD_ROW6 1
#define CONFIG_KEYPAD_ROW7 1

#define CONFIG_KEYPAD_COL0 1
#define CONFIG_KEYPAD_COL1 1
#define CONFIG_KEYPAD_COL2 1
#define CONFIG_KEYPAD_COL3 1
#define CONFIG_KEYPAD_COL4 1
#define CONFIG_KEYPAD_COL5 1
#define CONFIG_KEYPAD_COL6 1
#define CONFIG_KEYPAD_COL7 1

#define CONFIG_KEYPAD_ROW0_PIN 5
#define CONFIG_KEYPAD_ROW1_PIN 6
#define CONFIG_KEYPAD_ROW2_PIN 7
#define CONFIG_KEYPAD_ROW3_PIN 18
#define CONFIG_KEYPAD_ROW4_PIN 17
#define CONFIG_KEYPAD_ROW5_PIN 16
#define CONFIG_KEYPAD_ROW6_PIN 15
#define CONFIG_KEYPAD_ROW7_PIN 14

#define CONFIG_KEYPAD_COL0_PIN 22
#define CONFIG_KEYPAD_COL1_PIN 23
#define CONFIG_KEYPAD_COL2_PIN 27
#define CONFIG_KEYPAD_COL3_PIN 28
#define CONFIG_KEYPAD_COL4_PIN 2
#define CONFIG_KEYPAD_COL5_PIN 8
#define CONFIG_KEYPAD_COL6_PIN 9
#define CONFIG_KEYPAD_COL7_PIN 4

#define CONFIG_KEYPAD_COL0_PORTA 0
#define CONFIG_KEYPAD_COL1_PORTA 0
#define CONFIG_KEYPAD_COL2_PORTA 1
#define CONFIG_KEYPAD_COL3_PORTA 1
#define CONFIG_KEYPAD_COL4_PORTA 1
#define CONFIG_KEYPAD_COL5_PORTA 0
#define CONFIG_KEYPAD_COL6_PORTA 0
#define CONFIG_KEYPAD_COL7_PORTA 1

#define CONFIG_KEYPAD_ROW0_PORTA 1
#define CONFIG_KEYPAD_ROW1_PORTA 1
#define CONFIG_KEYPAD_ROW2_PORTA 1
#define CONFIG_KEYPAD_ROW3_PORTA 1
#define CONFIG_KEYPAD_ROW4_PORTA 1
#define CONFIG_KEYPAD_ROW5_PORTA 1
#define CONFIG_KEYPAD_ROW6_PORTA 1
#define CONFIG_KEYPAD_ROW7_PORTA 1

#define KEYPAD_SCAN_ROWS 0

#endif
