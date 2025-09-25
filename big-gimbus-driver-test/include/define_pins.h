#ifndef DEFINE_PINS_H
#define DEFINE_PINS_H

// This is an include guard. It prevents the compiler from including the
// contents of this file more than once, which can cause errors.

// Indicator LED
#define IND_LIGHT 42

// CAN bus flip switches
#define can_flip_left 17
#define can_flip_right 18
#define can_tx 38
#define can_rx 37

// I2C bus pins
#define i2c_sda 35
#define i2c_scl 36

// USB current sense pins
#define isen_usb_left 1
#define isen_usb_right 2

// Motor phase current sense pins
#define isen_u 5
#define isen_v 4
#define isen_w 6

// Encoder SPI pins
#define enc_cs 16
#define enc_scl 15
#define enc_sda 7

// Motor driver PWM pins
#define UH 8
#define UL 9
#define VH 10
#define VL 11
#define WH 12
#define WL 13

// Motor controller SPI pins
#define mc_cs 14
#define mc_clk 21
#define mc_cipo 48
#define mc_copi 47

#endif // DEFINE_PINS_H