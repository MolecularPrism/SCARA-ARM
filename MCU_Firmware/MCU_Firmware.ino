
#include <stdio.h>
#include <string.h>
#include <stdlib.h> 

// PID controller gains
#define Kp 1.0
#define Ki 0.1
#define Kd 0.01

// WSF Constants
#define WSF_SAMPLE_COUNT 7 
const double WSF_CONST_LOOKUP_TABLE[7] = {100, 51.3417, 26.3597, 13.53353, 6.948345, 3.567399, 1.831564};
#define WST_CONST_SUM 203.582238

// Define maximum and minimum output values
#define MAX_OUTPUT 100.0
#define MIN_OUTPUT 100.0


//pin def
const int trigPin = 13;
const int echoPin = 12;
const int pwmPin1 = 8;

//WSF variables
double raw_deriv_samples[WSF_SAMPLE_COUNT] = {};
double filtered_derivative;
int count = 0;


//variable def
long duration;
int distance;
double desired_position = 100.0; // Desired position of the robotic arm
double current_position;        // Current position of the robotic arm
double integral = 0.0, derivative, previous_error = 0.0;
double control_signal_output;
int saturation = 0;
int error;
unsigned long prev_time = 0;


//Timer1 ISR variables
double period_us1;
volatile bool pwmState1 = false;
volatile unsigned long previousMillis1 = 0;
volatile unsigned long onTime_ms1;
volatile unsigned long offTime_ms1;
volatile float dutyCycle_fraction1;  // Duty cycle in percentage
volatile unsigned long time_buffer_ms1 = offTime_ms1;

void PID_Controller(int error, int* saturation);



void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  pinMode(pwmPin1, OUTPUT);

  period_us1 = 4000 + 120;


  // Configure Timer1 for PWM generation
  noInterrupts();           // Disable interrupts during setup
  TCCR1A = 0;               // Clear Timer1 control registers
  TCCR1B = 0;
  TCNT1  = 0;               // Timer1 counter starts from 0
  OCR1A = (period_us1 * 16) / 256 - 1;  // Set PWM period
  TCCR1B |= (1 << WGM12);   // Configure Timer1 for CTC mode
  TCCR1B |= (1 << CS12);    // Set Timer1 prescaler to 256
  TCCR1B |= (0 << CS11);
  TCCR1B |= (0 << CS10);
  TIMSK1 |= (1 << OCIE1A);  // Enable Timer1 compare match interrupt
  interrupts();             // Enable interrupts

}

// Timer ISR (Interrupt Service Routine)
ISR(TIMER1_COMPA_vect) {
  digitalWrite(pwmPin1, HIGH);
  PID_Controller(error, &saturation);
  if(saturation){
    integral = 0.0;
  }

  OCR1A = (period_us1 * 16) / 256 - 1;

  digitalWrite(pwmPin1, LOW);

}

double FDD_WSF(double raw_deriv_samples[]){
  double sum = 0.0;
  double filtered_Derivative;

  for(int i = 0; i < WSF_SAMPLE_COUNT; i++){
    sum += WSF_CONST_LOOKUP_TABLE[i] * raw_deriv_samples[i];
  }
  filtered_Derivative = sum / WST_CONST_SUM;

  return filtered_Derivative;

  
}


//for testing purpose, we are going to pass in error
void PID_Controller(int error, int* saturation){

  // Calculate error
  //error = desired_position - current_position;

  unsigned long current_time = millis(); // Current time
  double dt = (current_time - prev_time) / 1000.0; // Time difference in seconds

  // Update integral term
  integral += error * dt;

  // Update derivative term
  derivative = (error - previous_error) / dt;

  raw_deriv_samples[count] = derivative;
  count++;

  if(count == WSF_SAMPLE_COUNT){ 
    filtered_derivative = FDD_WSF(raw_deriv_samples);

    // Calculate control signal
    control_signal_output = Kp * error + Ki * integral + Kd * filtered_derivative;

    // Limit control signal output (prevent integration winding)
    if (control_signal_output > MAX_OUTPUT) {
        *saturation = 1; // Set saturation flag
        control_signal_output = MAX_OUTPUT;
    } else if (control_signal_output < MIN_OUTPUT) {
        *saturation = 1; // Set saturation flag
        control_signal_output = MIN_OUTPUT;
    } else {
        *saturation = 0; // Clear saturation flag
    }

    //reset count
    count = 0;

  }

  // Update previous error for derivative
  previous_error = error;
  prev_time = current_time;

}

int analog_to_period(double analog){
  int period_ms = analog * 5;

  return period_ms;
}

void loop() {

  for(int i = 0; i<1000; i++){
    Serial.print("\n");
    Serial.print("i is: ");
    Serial.print(i);
  }


}