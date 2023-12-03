#include "mbed.h"
#include "rtos.h"
#include "EthernetInterface.h"
#include "ExperimentServer.h"
#include "QEI.h"
#include "MotorShield.h" 
#include "HardwareSetup.h"

#define NUM_INPUTS 17
#define NUM_OUTPUTS 11

// (motor) constants 
float pi = 3.1415926;
float km = 0.5;
float kb = 0.16;
float constraint_angle = pi/2;
float duty_factor = 1.0;

// Measured Values
// MOTOR 1
float velocity1 = 0;
float current1 = 0;
float theta1 = 0;

// MOTOR 2
float velocity2 = 0;
float current2 = 0;
float theta2 = 0;

// Set values for PID 
float current1_desired = 0;
float current2_desired = 0;
float Kp_1 = 4.0f;
float Ki_1 = 0.4f;
float Kd_1 = 0.0;
float Kp_2 = 4.0f;
float Ki_2 = 0.4f;
float Kd_2 = 0.0;

// Controller values
float voltage1 = 0;
float voltage2 = 0;
float duty1 = 0;
float duty2 = 0;

float R = 12.0/3.1;
float error1 = 0;
float error2 = 0;
float total_error1 = 0;
float total_error2 = 0;
float tau_d1 = 0;
float tau_d2 = 0;
float b = .00032;
float theta1_init = 0;
float theta2_init = 0;

// Spring Coefficients for Motors
float K_1 = 0;
float D_1 = 0;
float K_2 = 0;
float D_2 = 0;

float desired_forearm = 0;
float desired_hand = 0;

Serial pc(USBTX, USBRX);    // USB Serial Terminal
ExperimentServer server;    // Object that lets us communicate with MATLAB
Timer t;                    // Timer to measure elapsed time of experiment
Ticker ControlLoop;         // Ticker to run current controller at high frequency

QEI encoderA(PE_9,PE_11, NC, 1200, QEI::X4_ENCODING);  // MOTOR A ENCODER (no index, 1200 counts/rev, Quadrature encoding)
QEI encoderB(PA_5, PB_3, NC, 1200, QEI::X4_ENCODING);  // MOTOR B ENCODER (no index, 1200 counts/rev, Quadrature encoding)
//QEI encoderC(PC_6, PC_7, NC, 1200, QEI::X4_ENCODING);  // MOTOR C ENCODER (no index, 1200 counts/rev, Quadrature encoding)
//QEI encoderD(PD_12, PD_13, NC, 1200, QEI::X4_ENCODING);// MOTOR D ENCODER (no index, 1200 counts/rev, Quadrature encoding)

MotorShield motorShield(24000); // initialize the motor shield with a period of 12000 clock ticks or ~10kHZ

// function to calculate motor voltage according to current control law
void current_control() {
    // MOTOR A
    theta1 = encoderA.getPulses()*(6.2831/1200.0) + theta1_init;
    velocity1 = encoderA.getVelocity()*(6.2831/1200.0);
    current1 = -(motorShield.readCurrentA()*(30.0/65536.0)-15.0); //read current for motor A in amps. Note: this is a slightly different current sensor so its a different conversion than last lab.            
    error1 = current1_desired - current1;
    total_error1 = total_error1 + error1;

    if (total_error1 > 3000){
        total_error1 = 3000;
    } else if (total_error1 < -3000) {
        total_error1 = -3000;
    }

    // MOTOR A - PI CONTROLLER
    // voltage1 = R*current1_desired + Kp_1*(current1_desired - current1) + Ki_1*total_error1 + kb*velocity1;
    
    // MOTOR A - PID CONTROLLER
    voltage1 = R*current1_desired + Kp_1*(current1_desired - current1) + Ki_1*total_error1 - Kd_1*velocity1 + kb*velocity1;

    // MOTOR B
    theta2 = theta1 + encoderB.getPulses()*(6.2831/1200.0) + theta2_init;
    velocity2 = encoderB.getVelocity()*(6.2831/1200.0);
    current2 = -(motorShield.readCurrentB()*(30.0/65536.0)-15.0); //read current for motor A in amps. Note: this is a slightly different current sensor so its a different conversion than last lab.            
    error2 = current2_desired - current2;
    total_error2 = total_error2 + error2;

    if (total_error2 > 3000){
        total_error2 = 3000;
    } else if (total_error2 < -3000) {
        total_error2 = -3000;
    }

    // MOTOR B - PI CONTROLLER
    // voltage2 = R*current2_desired + Kp_2*(current2_desired - current2) + Ki_2*total_error2 + kb*velocity2;

    // MOTOR B - PID CONTROLLER
    voltage2 = R*current2_desired + Kp_2*(current2_desired - current2) + Ki_2*total_error2 - Kd_2*velocity2 + kb*velocity2;

    duty1  = voltage1/12.0 * duty_factor;
    if (duty1 > 1) {
        duty1 = 1;
    }
    if (duty1 < -1) {
        duty1 = -1;
    }

    if (duty1 >= 0){
        motorShield.motorAWrite(duty1, 0);
    }
    else if (duty1 < 0){
        motorShield.motorAWrite(abs(duty1), 1);
    }

    duty2  = voltage2/12.0 * duty_factor;
    if (duty2 > 1) {
        duty2 = 1;
    }
    if (duty2 < -1) {
        duty2 = -1;  
    }

    if (duty2 >= 0){
        motorShield.motorBWrite(duty2, 0);
    }
    else if (duty2 < 0){
        motorShield.motorBWrite(abs(duty2), 1);
    }
}


int main (void) {
    // Link the terminal with our server and start it up
    server.attachTerminal(pc);
    server.init();
   
    // Define array to hold input parameters from MATLAB
    float input_params[NUM_INPUTS];
   
    while(1) {
        // Run experiment every time input parameters are received from MATLAB
        if (server.getParams(input_params, NUM_INPUTS)) {
            // Unpack inputs
            // MOTOR 1 PID CONTROLLER
            Kp_1 = input_params[0];
            Ki_1 = input_params[1];
            Kd_1 = input_params[14];
            current1_desired = input_params[2];

            // MOTOR 1 SPRING COEFFICIENTS
            K_1 = input_params[3];
            D_1 = input_params[4];

            // MOTOR 2 PID CONTROLLER
            Kp_2 = input_params[5];
            Ki_2 = input_params[6];
            Kd_2 = input_params[15];
            current2_desired = input_params[7];

            // MOTOR 2 SPRING COEFFICIENTS
            K_2 = input_params[8];
            D_2 = input_params[9];

            // ANGLE
            desired_forearm = input_params[10];
            theta1_init = input_params[11]; 
            theta2_init = input_params[12]; 
            desired_hand = input_params[13];

            // MISCELLANEOUS
            duty_factor = input_params[16];

            // Run current controller at 10kHz
            ControlLoop.attach(&current_control,0.0001);

            // Setup experiment
            t.reset();
            t.start();
            encoderA.reset();
            encoderB.reset();
            // encoderC.reset();
            // encoderD.reset();

            motorShield.motorAWrite(0, 0); //turn motor A off
            motorShield.motorBWrite(0, 0); //turn motor B off
            // Use the motor shield as follows:
            // motorShield.motorAWrite(DUTY CYCLE, DIRECTION), DIRECTION = 0 is forward, DIRECTION = 1 is backwards.

            // Run experiment
            while (t.read() < 5) {
                // Perform impedance control loop logic to calculate desired current
                tau_d1 = K_1*(desired_forearm - theta1) - D_1*velocity1;
                // tau_d1 = -K_1*theta1 - D_1*velocity1;
                current1_desired = tau_d1/kb; // Set commanded current from impedance controller here.

                tau_d2 = K_2*(desired_hand - theta2) - D_2*velocity2;
                // tau_d2 = -K_2*theta2 - D_2*velocity2;
                current2_desired = tau_d2/kb;

                // THIS IS THE HARDSTOPPPPPPPP 
                // THIS HAS NOT BEEN TESTED YET -- PLZ TEST WITH CAUTION 
                // should have hit the ball once get to 3*pi/4 
                // resets desired angle to 0 
                // if (theta1 > encoderA.getPulses()*(6.2831/1200.0) + 3*pi/4) {
                //    current1_desired = -km*constraint_angle/kb;
                //    tau_d1 = current1_desired*kb; 
                //}

                // Send data to MATLAB
                float output_data[NUM_OUTPUTS];

                output_data[0] = t.read();

                // MOTOR A (ARM) DATA
                output_data[1] = theta1;
                output_data[2] = velocity1;
                output_data[3] = current1;
                output_data[4] = voltage1;
                output_data[5] = tau_d1;

                // MOTOR B (HAND) DATA
                output_data[6] = theta2;
                output_data[7] = velocity2;
                output_data[8] = current2;
                output_data[9] = voltage2;
                output_data[10] = tau_d2;

                server.sendData(output_data, NUM_OUTPUTS);              
                ThisThread::sleep_for(1); //run outer control loop at 1kHz
            }

            // Cleanup after each experiment
            ControlLoop.detach();
            server.setExperimentComplete();
            motorShield.motorAWrite(0, 0); //turn motor A off
            motorShield.motorBWrite(0, 0); //turn motor A off
        } // end if
    } // end while

} // end main