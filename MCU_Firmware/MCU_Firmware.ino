// PID controller gains
#define Kp 1.0
#define Ki 0.1
#define Kd 0.01

// Define maximum and minimum output values
#define MAX_OUTPUT 100.0
#define MIN_OUTPUT 0.0


//pin def
const int trigPin = 13;
const int echoPin = 12;
const int pwmPin1 = 11;


//variable def
long duration;
int distance;
double desired_position = 90.0; // Desired position of the robotic arm
double current_position;        // Current position of the robotic arm
double integral = 0.0, derivative, previous_error = 0.0;
double control_signal_output;
int saturation = 0;

//Timer1 ISR variables
int period_ms1 = 100;
volatile bool pwmState1 = false;
volatile unsigned long previousMillis1 = 0;
volatile unsigned long onTime_ms1;
volatile unsigned long offTime_ms1;
volatile float dutyCycle_fraction1;  // Duty cycle in percentage
volatile unsigned long time_buffer_ms1 = offTime_ms1;

double PID_Controller(int error, int* saturation);



void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  pinMode(pwmPin1, OUTPUT);

  // Calculate on-time and off-time based on duty cycle
  onTime_ms1 = period_ms1 * dutyCycle_fraction1;
  offTime_ms1 = period_ms1 - onTime_ms1;

  // Configure Timer1 for PWM generation
  noInterrupts();           // Disable interrupts during setup
  TCCR1A = 0;               // Clear Timer1 control registers
  TCCR1B = 0;
  TCNT1  = 0;               // Timer1 counter starts from 0
  OCR1A = (16000 * offTime_ms1) / 1000 - 1;  // Set PWM period
  TCCR1B |= (1 << WGM12);   // Configure Timer1 for CTC mode
  TCCR1B |= (1 << CS12);    // Set Timer1 prescaler to 8 (timer counts 1/8th of clock speed)
  TIMSK1 |= (1 << OCIE1A);  // Enable Timer1 compare match interrupt
  interrupts();             // Enable interrupts

}

// Timer ISR (Interrupt Service Routine)
ISR(TIMER1_COMPA_vect) {
  
  // Update OCR1A to switch off after the on time
  OCR1A = (16000 * offTime_ms1 * pwmState1 + 16000 * onTime_ms1 * (!pwmState1)) / 1000 - 1;


  // Toggle PWM state if duty cycle is not 0

  pwmState1 = !pwmState1;
  digitalWrite(pwmPin1, pwmState1);

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
  delay(20);

  return distance;
}

//for testing purpose, we are going to pass in error
double PID_Controller(int error, int* saturation){

  // Calculate error
  //error = desired_position - current_position;


  // Update integral term
  integral += error;

  // Update derivative term
  derivative = error - previous_error;

  // Calculate control signal
  control_signal_output = Kp * error + Ki * integral + Kd * derivative;

   // Limit control signal output
    if (control_signal_output > MAX_OUTPUT) {
        *saturation = 1; // Set saturation flag
        control_signal_output = MAX_OUTPUT;
    } else if (control_signal_output < MIN_OUTPUT) {
        *saturation = 1; // Set saturation flag
        control_signal_output = MIN_OUTPUT;
    } else {
        *saturation = 0; // Clear saturation flag
    }

  // Update previous error for derivative
  previous_error = error;
  Serial.print("\nanalog output: ");
  Serial.print(control_signal_output);
  Serial.print(" at ");
  Serial.print(error);
  Serial.print(" cm\n");


  return control_signal_output;
}

void loop() {
  int error = ultrasonic_handler();
  int analog_out = PID_Controller(error, &saturation);

  //correlate analog_out of PID controller with duty cycle
  dutyCycle_fraction1 = analog_out / 100.1;

  // Recalculate on-time and off-time based on updated duty cycle fraction
  onTime_ms1 = period_ms1 * dutyCycle_fraction1;
  offTime_ms1 = period_ms1 - onTime_ms1;

  if(saturation){
    integral = 0.0;
  }

}