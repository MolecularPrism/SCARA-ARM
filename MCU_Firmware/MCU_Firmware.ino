#include <stdio.h>
#include <string.h>
#include <stdlib.h>

// Desired Angle
volatile int desired_angle = 90;

// Pins
const int bit0 = 10;
const int bit1 = 4;
const int bit2 = 5;
const int bit3 = 6;
const int bit4 = 7;
const int bit5 = 8;
const int bit6 = 9;
const int bit7 = 2;

const int pwmLeftPin = 3;    // Counterclockwise
const int pwmRightPin = 11;  // Clockwise

// For decoding
volatile int bit0_state;
volatile int bit1_state;
volatile int bit2_state;
volatile int bit3_state;
volatile int bit4_state;
volatile int bit5_state;
volatile int bit6_state;
volatile int bit7_state;

const int MAX_DECODER_VALUE = 89;

volatile int decoder_deg_sign = 1;  // 1 is right 0 is left

volatile int current_decoder_val = 0;
volatile int previous_decoder_val = 0;
volatile int rotations = 0;  // Total 90 deg rotations

volatile int absolute_angle = 0;

// For PID
volatile float k = 1.1;
volatile float kp = 0.2;  // Increasing speeds up response and removes overshoot
volatile float ki = 0.90;  // Increasing removes steady state error
volatile float kd = 0.01;    // Increasing slows down response and removes spikes
volatile float integral_max = 10;
volatile float integral_min = -10;

volatile int error = 0;
volatile float error_derivative = 0;
volatile float error_integral = 0;
volatile float error_previous = 0;
volatile float pid_out = 0;
volatile float new_error_integral;
volatile float filtered_error_derivative = 0;

volatile long current_time = 0;
volatile long previous_time = 0;
volatile float delta_time = 0;

// For WSF
volatile float raw_deriv_samples[10] = { 0, 0, 0, 0, 0, 0, 0 };
volatile int raw_angle_samples[10] = { 0, 0, 0, 0, 0, 0, 0 };
const int WSF_SAMPLE_COUNT = 10;
const float WSF_CONST_LOOKUP_TABLE[10] = { 0.36307, 0.2327935, 0.149263, 0.095704, 0.0613637, 0.0393452, 0.0252274, 0.0161753, 0.0103713, 0.006649864 };

// Etc.
volatile float sum = 0.0;

volatile int motor_deg_sign = 1;

volatile int motor_speed = 0;


void setup() {
  Serial.begin(9600);

  // Decoder Input Pins
  pinMode(bit0, INPUT);
  pinMode(bit1, INPUT);
  pinMode(bit2, INPUT);
  pinMode(bit3, INPUT);
  pinMode(bit4, INPUT);
  pinMode(bit5, INPUT);
  pinMode(bit6, INPUT);
  pinMode(bit7, INPUT);

  // Output Pins
  pinMode(pwmLeftPin, OUTPUT);
  pinMode(pwmRightPin, OUTPUT);

  // Setup Timer 1
  noInterrupts();  // Disable all interrupts

  // Timer1 Setup
  TCCR1A = 0;
  TCCR1B = 0;
  TCNT1 = 0;
  OCR1A = 1599;              // Set for 1.25kHz Control Freq
  TCCR1B |= (1 << WGM12);   // CTC mode
  TCCR1B |= (1 << CS11);    // Prescaler 8
  TIMSK1 |= (1 << OCIE1A);  // Enable Timer1 interrupt

  // Timer2 Setup
  TCCR2A = 0;  // Clear Timer2 control register A
  TCCR2B = 0;  // Clear Timer2 control register B
  TCNT2 = 0;   // Initialize counter value to 0

  // Set to Fast PWM mode
  TCCR2A |= (1 << WGM21) | (1 << WGM20);

  // Set prescaler to 8 (or choose another suitable prescaler)
  TCCR2B |= (1 << CS21);

  // Non-inverting mode for both OC2A and OC2B
  TCCR2A |= (1 << COM2A1) | (1 << COM2B1);

  OCR2A = 0;
  OCR2B = 0;

  interrupts();  // Enable all interrupts

  // Read initial decoder val
  readDecoder();
  previous_decoder_val = current_decoder_val;
}

void readDecoder() {
  bit0_state = (PINB & (1 << PB2)) >> PB2;
  bit1_state = (PIND & (1 << PD4)) >> PD4;
  bit2_state = (PIND & (1 << PD5)) >> PD5;
  bit3_state = (PIND & (1 << PD6)) >> PD6;
  bit4_state = (PIND & (1 << PD7)) >> PD7;
  bit5_state = (PINB & (1 << PB0)) >> PB0;
  bit6_state = (PINB & (1 << PB1)) >> PB1;
  bit7_state = (PIND & (1 << PD2)) >> PD2;

  binaryToDecimal();

  decoder_deg_sign = bit7_state;
}

void binaryToDecimal() {
  current_decoder_val = bit6_state * 64 + bit5_state * 32 + bit4_state * 16 + bit3_state * 8 + bit2_state * 4 + bit1_state * 2 + bit0_state;
}

void getAbsolutePosition() {
  readDecoder();

  // Determine direction and handle overflows/underflows
  if (decoder_deg_sign == 1) {  // clockwise
    if (previous_decoder_val > current_decoder_val) {
      // Handle overflow
      rotations++;
    }
  } else {  // counterclockwise
    if (previous_decoder_val < current_decoder_val) {
      // Handle underflow
      rotations--;
    }
  }

  previous_decoder_val = current_decoder_val;

  absolute_angle = rotations * (MAX_DECODER_VALUE + 1) + current_decoder_val;
}

void setMotorSpeed() {
  // Get motor speed
  motor_speed = fabs(pid_out);
  if (motor_speed > 255) {
    motor_speed = 255;
  }

  // Get motor direction
  if (pid_out > 0) {  // motor needs to move clockwise
    motor_deg_sign = 1;
  } else {
    // motor needs to move counterclockwise
    motor_deg_sign = 0;
  }

  if (motor_deg_sign == 1) {
    OCR2A = motor_speed;
    OCR2B = 0;
  } else {
    OCR2A = 0;
    OCR2B = motor_speed;
  }
}

void pidControl() {
  // time difference
  current_time = micros();

  delta_time = ((float)(current_time - previous_time))/1.0e6; // in seconds
  previous_time = current_time;

  // error
  error = desired_angle - absolute_angle;  

  // derivative
  error_derivative = (error - error_previous) / (delta_time);

  FDD_WSF_PID();

  // integral
  error_integral = error_integral + error * delta_time;

  if(error_integral > integral_max){
    error_integral = integral_max;
  }
  else if(error_integral < integral_min){
    error_integral = integral_min;
  }

  // control signal
  pid_out = k * (kp * error + kd * filtered_error_derivative + ki * error_integral);

}

ISR(TIMER1_COMPA_vect) {
  // Get new desired position

  // Read from decoder and get absolute position
  getAbsolutePosition();

  // PID Computation
  pidControl();

  // Set Motor Speed
  setMotorSpeed();
}

void motorDirectionControl() {
  if (motor_deg_sign == 1) {
    PORTD &= ~(1 << PD3);  // Low
    PORTB |= (1 << PB3);   // High
  } else {
    PORTB &= ~(1 << PB3);  // Low
    PORTD |= (1 << PD3);   // High
  }
}

ISR(TIMER2_COMPA_vect) {

  // Motor Direction Control
  motorDirectionControl();

  // Store previous error
  error_previous = error;
}

void FDD_WSF_PID() {
    sum = 0.0;

    // Shift all the derivative samples to the right by 1 index
    for (int i = WSF_SAMPLE_COUNT - 1; i > 0; i--) {
        raw_deriv_samples[i] = raw_deriv_samples[i - 1];
    }

    // Insert new derivative data to the first index
    raw_deriv_samples[0] = error_derivative;

    // Sum all of them
    for (int i = 0; i < WSF_SAMPLE_COUNT; i++) {
        sum += raw_deriv_samples[i] * WSF_CONST_LOOKUP_TABLE[i];
    }

    // Update filtered error derivative
    filtered_error_derivative = sum;
}

void loop() {

  Serial.println(filtered_error_derivative);
}