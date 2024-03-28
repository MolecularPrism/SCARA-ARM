
#include <stdio.h>
#include <string.h>
#include <stdlib.h> 
#include <PID_v1_bc.h>

// PID controller gains
float Kp = 1.0;
float Ki = 1.0;
float Kd = 1.0;

// WSF Constants
#define WSF_SAMPLE_COUNT 7 
const double WSF_CONST_LOOKUP_TABLE[7] = {100, 51.3417, 26.3597, 13.53353, 6.948345, 3.567399, 1.831564};
#define WST_CONST_SUM 203.582238

// Define maximum and minimum output values
#define MAX_OUTPUT 10.0
#define MIN_OUTPUT -10.0



//pin def
const int trigPin = 13;
const int echoPin = 12;
const int pwmPin1 = 11;
const int pwmPin2 = 10;

const int bit7 = 9;
const int bit6 = 8;
const int bit5 = 7;
const int bit4 = 6;
const int bit3 = 5;
const int bit2 = 4;
const int bit1 = 3;
const int bit0 = 2;

//WSF variables
double raw_deriv_samples[WSF_SAMPLE_COUNT] = {0, 0, 0, 0, 0, 0, 0};
double filtered_derivative;

//Decoder Variables
int current_deg = 0;

//Timer1 ISR variables
volatile double period_us1;

volatile int bit0_state;
volatile int bit1_state;
volatile int bit2_state;
volatile int bit3_state;
volatile int bit4_state;
volatile int bit5_state;
volatile int bit6_state;
volatile int bit7_state;


void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  pinMode(pwmPin1, OUTPUT);
  pinMode(pwmPin2, OUTPUT);
  pinMode(bit0, INPUT);
  pinMode(bit1, INPUT);
  pinMode(bit2, INPUT);
  pinMode(bit3, INPUT);
  pinMode(bit4, INPUT);
  pinMode(bit5, INPUT);
  pinMode(bit6, INPUT);
  pinMode(bit7, INPUT)

  // Set trigger pin as an output and echo pin as an input
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);

  period_us1 = 0 + onTime; //offtime + ontime

  // Configure Timer1 for PWM generation
  noInterrupts();           // Disable interrupts during setup
  TCCR1A = 0;               // Clear Timer1 control registers
  TCCR1B = 0;
  TCNT1  = 0;               // Timer1 counter starts from 0
  OCR1A = (period_us1 * 16) /8  - 1;  // Set PWM period
  TCCR1B |= (1 << WGM12);   // Configure Timer1 for CTC mode
  TCCR1B |= (0 << CS12);    // Set Timer1 prescaler to 8
  TCCR1B |= (1 << CS11);
  TCCR1B |= (0 << CS10);
  TIMSK1 |= (1 << OCIE1A);  // Enable Timer1 compare match interrupt

  interrupts();             // Enable interrupts

}


ISR(TIMER1_COMPA_vect) {
  update_current_deg();

}


void update_current_deg() {
  bit0_state = digitalRead(bit0);
  bit1_state = digitalRead(bit1);
  bit2_state = digitalRead(bit2);
  bit3_state = digitalRead(bit3);
  bit4_state = digitalRead(bit4);
  bit5_state = digitalRead(bit5);
  bit6_state = digitalRead(bit6);
  bit7_state = digitalRead(bit7);

  current_deg = binaryToDecimal(bit0_state, bit1_state, bit2_state, bit3_state, bit4_state, bit5_state, bit6_state);

  if(!bit7_state){
    current_deg = current_deg - 90;
  }

  
}

int binaryToDecimal(int bit0, int bit1, int bit2, int bit3, int bit4, int bit5, int bit6) { //done
  return bit6 * 64 + bit5 * 32 + bit4 * 16 + bit3 * 8 + bit2 * 4 + bit1 * 2 + bit0;
}

void loop() {
  // put your main code here, to run repeatedly:
  // Control motor
  //moveMotor(u);

}
