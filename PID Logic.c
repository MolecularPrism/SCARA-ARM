#include <stdio.h>

// PID controller gains
#define Kp 1.0
#define Ki 0.1
#define Kd 0.01

double read_current_position(){
    //PLD will send a digital signal to the MCU and we process the signal and convert to current position
}

double get_desired_position(){
    //we get the desired position from the 3D Printer "Cheat Code"
}

int main() {
    double desired_position = 90.0; // Desired position of the robotic arm
    double current_position;        // Current position of the robotic arm
    double error, integral = 0.0, derivative, previous_error = 0.0;
    double control_signal_output;

    while (1) {
        // Read current position
        current_position = read_current_position();

        // Calculate error
        error = desired_position - current_position;

        // Update integral term
        integral += error;

        // Update derivative term
        derivative = error - previous_error;

        // Calculate control signal
        control_signal_output = Kp * error + Ki * integral + Kd * derivative;

        // Update previous error for derivative
        previous_error = error;

    }

    return 0;
}
