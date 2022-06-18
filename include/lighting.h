
#pragma once
#include <stdint.h> // needed for uint_8t indentifier
#define FASTLED_INTERNAL // disable pragma messages from FastLED
#include <FastLED.h> // wanted for EVERY_N_MILLISECONDS function

#include <OctoWS2811.h>
#include <Adafruit_NeoPixel.h>
#ifdef __AVR__
  #include <avr/power.h>
#endif


#define LV1    14
#define LV2    15
#define LV3    22
#define LV4    23

//#define CLK_PIN   32
#define NUM_LEDS    9 // 9 leds on each lightbar. totall is 18








float breathe(); // breathing exponential curve generator
