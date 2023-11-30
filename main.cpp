#include "mbed.h"
#include "rtos.h"
#include "EthernetInterface.h"
#include "ExperimentServer.h"
#include "QEI.h"
#include "MotorShield.h" 
#include "HardwareSetup.h"

#define NUM_INPUTS 16
#define NUM_OUTPUTS 11

// (motor) constants 
float pi = 3.1415926;
float km = 0.5;
float kb = 0.16;
float constraint_angle = pi/2;

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
float current_d1 = 0;
float current_d2 = 0;
float current_Kp = 4.0f;         
float current_Ki = 0.4f;   
float kp1 = 4.0f;
float ki1 = 0.4f;
float kd1 = 0.0;
float kp2 = 4.0f;
float ki2 = 0.4f;
float kd2 = 0.0;

// Controller values
float volt1 = 0;
float volt2 = 0;
float duty1 = 0;
float duty2 = 0;

float R = 3.5;
float error1 = 0;
float error2 = 0;
float total_error1 = 0;
float total_error2 = 0;
float tau_d1 = 0;
float tau_d2 = 0;
float b = .00032;
float t1_i = 0;
float t2_i = 0;

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
    theta1 = encoderA.getPulses()*(6.2831/1200.0) + t1_i;
    velocity1 = encoderA.getVelocity()*(6.2831/1200.0);
    current1 = -(motorShield.readCurrentA()*(30.0/65536.0)-15.0); //read current for motor A in amps. Note: this is a slightly different current sensor so its a different conversion than last lab.            
    error1 = current_d1 - current1;
    total_error1 = total_error1 + error1;

    if (total_error1 > 3000){
        total_error1 = 3000;
    } else if (total_error1 < -3000) {
        total_error1 = -30000;
    }

    // PI CONTROLLER
    // volt1 = R*current_d1 + kp1*(current_d1 - current1) + ki1*total_error1 + kb*velocity1;
    
    // PID CONTROLLER
    volt1 = R*current_d1 + kp1*(current_d1 - current1) + ki1*total_error1 - kd1*velocity1 + kb*velocity1;

    // MOTOR B
    theta2 = theta1 + encoderB.getPulses()*(6.2831/1200.0) + t2_i;
    velocity2 = encoderB.getVelocity()*(6.2831/1200.0);
    current2 = -(motorShield.readCurrentB()*(30.0/65536.0)-15.0); //read current for motor A in amps. Note: this is a slightly different current sensor so its a different conversion than last lab.            
    error2 = current_d2 - current2;
    total_error2 = total_error2 + error2;

    if (total_error2 > 3000){
        total_error2 = 3000;
    } else if (total_error2 < -3000) {
        total_error2 = -30000; 
    }

    // PI CONTROLLER
    // volt2 = R*current_d2 + kp2*(current_d2 - current2) + ki2*total_error2 + kb*velocity2;

    // PID CONTROLLER
    volt2 = R*current_d2 + kp2*(current_d2 - current2) + ki2*total_error2 - kd2*velocity2 + kb*velocity2;

    duty1  = volt1/12.0;
    if (duty1 >  1) {
        duty1 =  1;
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

    duty2  = volt2/12.0;
    if (duty2 >  1) {
        duty2 =  1;
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
    pc.printf("%f", input_params[0]);
   
    while(1) {
        // Run experiment every time input parameters are received from MATLAB
        if (server.getParams(input_params, NUM_INPUTS)) {
            // Unpack inputs
            // motor 1 PID 
            kp1 = input_params[0];
            ki1 = input_params[1];
            kd1 = input_params[14];
            current_d1 = input_params[2];

            // motor 1 spring coefficients 
            K_1 = input_params[3];
            D_1 = input_params[4];

            // motor 2 PID 
            kp2 = input_params[5];
            ki2 = input_params[6];
            kd2 = input_params[15];
            current_d2 = input_params[7];

            // motor 2 spring coefficients 
            K_2 = input_params[8];
            D_2 = input_params[9];

            // angle 
            desired_forearm = input_params[10];
            t1_i = input_params[11]; 
            t2_i = input_params[12]; 
            desired_hand = input_params[13];
            
            // Run current controller at 10kHz
            ControlLoop.attach(&current_control,0.0001);
           
            // Setup experiment
            t.reset();
            t.start();
            encoderA.reset();
            encoderB.reset();

            motorShield.motorAWrite(0, 0); //turn motor A off
            motorShield.motorBWrite(0, 0); //turn motor B off
            // Use the motor shield as follows:
            // motorShield.motorAWrite(DUTY CYCLE, DIRECTION), DIRECTION = 0 is forward, DIRECTION = 1 is backwards.
             
            // Run experiment
            while (t.read() < 5) {
                // Perform impedance control loop logic to calculate desired current
                // current_d = 0; // Set commanded current from impedance controller here.
                tau_d1 = K_1*(desired_forearm - theta1) - D_1*velocity1;
                current_d1 = tau_d1/kb; // Set commanded current from impedance controller here.

                tau_d2 = K_2*(desired_hand - theta2)  - D_2*velocity2;
                current_d2 = tau_d2/kb;

                // THIS IS THE HARDSTOPPPPPPPP 
                // THIS HAS NOT BEEN TESTED YET -- PLZ TEST WITH CAUTION 
                // should have hit the ball once get to 3*pi/4 
                // resets desired angle to 0 
                // if (theta1 > encoderA.getPulses()*(6.2831/1200.0) + 3*pi/4) {
                //    current_d1 = -km*constraint_angle/kb;
                //    tau_d1 = current_d1*kb; 
                //}
               
                // Send data to MATLAB
                float output_data[NUM_OUTPUTS];

                output_data[0] = t.read();

                // MOTOR A (ARM) DATA
                output_data[1] = theta1;
                output_data[2] = velocity1;
                output_data[3] = current1;
                output_data[4] = volt1;
                output_data[5] = tau_d1;

                // MOTOR B (HAND) DATA
                output_data[6] = theta2;
                output_data[7] = velocity2;
                output_data[8] = current2;
                output_data[9] = volt2;
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