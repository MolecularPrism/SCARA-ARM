#include <Arduino.h>

// PID controller gains
#define Kp 1.0
#define Ki 0.1
#define Kd 0.01

// Define maximum and minimum output values
#define MAX_OUTPUT 100.0
#define MIN_OUTPUT 0.0

// Pin definitions
const int trigPin = 13; // Example pin for trigger
const int echoPin = 12; // Example pin for echo
const int pwmPin1 = 11; // Example pin for PWM output

//Global variable
double integral = 0.0;
long duration;
int distance;
int saturation = 0;
int prev_error;
double analog_val;
unsigned long prev_time = 0;

//ISR variables
int period_ms1 = 90;

void setup() {
  Serial.begin(9600);
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  pinMode(pwmPin1, OUTPUT);

  // Configure Timer1 for PWM generation
  noInterrupts();           // Disable interrupts during setup
  TCCR1A = 0;               // Clear Timer1 control registers
  TCCR1B = 0;
  TCNT1  = 0;               // Timer1 counter starts from 0
  OCR1A = (16000 * period_ms1) / 1000 - 1;  // Set PWM period
  TCCR1B |= (1 << WGM12);   // Configure Timer1 for CTC mode
  TCCR1B |= (1 << CS12);    // Set Timer1 prescaler to 8 (timer counts 1/8th of clock speed)
  TIMSK1 |= (1 << OCIE1A);  // Enable Timer1 compare match interrupt
  interrupts();             // Enable interrupts
}

// Timer1 ISR (Interrupt Service Routine)
void timer1_ISR() {
  analog_val = PID_Controller(10, &saturation);

  if (saturation) {
    integral = 0.0;
  }
}

int ultrasonic_handler(){
  // Clear the trigPin
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  
  // Set the trigPin on HIGH state for 10 micro seconds
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  
  // Read the echoPin, and calculate the duration in microseconds
  duration = pulseIn(echoPin, HIGH);
  
  // Calculate the distance (in cm) based on the speed of sound (340 m/s)
  // and the time it took for the ultrasonic pulse to travel back and forth
  distance = duration * 0.034 / 2;

  // Delay before next measurement
  delay(100);

  return distance;
}



// PID Controller function
double PID_Controller(int error, int* saturation) {

  unsigned long current_time = millis(); // Current time
  double dt = (current_time - prev_time) / 1000.0; // Time difference in seconds

  // Your PID controller logic here
  double output_const = 0.1; //to convert from analog to Voltage

  integral += error;

  double P = Kp * error;
  double I = Ki * integral  * dt;
  double D = Kd * (error - prev_error) / dt; //rate of error change
  double output = output_const * (P + I + D);


  prev_error = error;
  prev_time = current_time;


  // Limit control signal output (prevent integration winding)
    if (output > MAX_OUTPUT) {
        *saturation = 1; // Set saturation flag
        output = MAX_OUTPUT;
    } else if (output < MIN_OUTPUT) {
        *saturation = 1; // Set saturation flag
        output = MIN_OUTPUT;
    } else {
        *saturation = 0; // Clear saturation flag
    }

    Serial.print("\nAnalog: ");
    Serial.print(output);

  return output;
}

int analog_to_period(double analog) {
  // Your analog to period conversion logic here
  int period = 100000;
  return period;

}

void loop() {

  period_ms1 = analog_to_period(analog_val);
  Serial.print(period_ms1);
}
