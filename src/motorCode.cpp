#include "motorCode.h"
#include <Arduino.h>


int ledPin = 13;

volatile int motor_a = 0;
volatile int motor_b = 0;
volatile int motor_c = 0;
volatile int motor_d = 0;

// Motor and Encoders ---------------------------------------------

// Input 0
int enA = 0; 
int dir1_a = 1;
int dir2_a = 2;
int e1a = 31; // encoder a, output 1
int e2a = 32; // encoder a, output 2

// Input 1
int enB = 5;
int dir1_b = 3;
int dir2_b = 4;
int e1b = 26;
int e2b = 25;

// Input 2
int enC = 8;
int dir1_c = 7;
int dir2_c = 6;
int e1c = 28;
int e2c = 27;

// Input 3
int enD = 12; 
int dir1_d = 10;
int dir2_d = 11;
int e1d = 33;
int e2d = 34;



