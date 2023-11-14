#include "mbed.h"
#include "rtos.h"
#include "EthernetInterface.h"
#include "ExperimentServer.h"
#include "QEI.h"
#include "MotorShield.h" 
#include "HardwareSetup.h"

#define NUM_INPUTS 13
#define NUM_OUTPUTS 9

#define PULSE_TO_RAD (2.0f*3.14159f / 1200.0f)

// variables 
float  pi = 3.1415; 

//motor 1 
float current_d1 = 0;
float kp = 0;
float ki = 0;
float velocity1 = 0;
float current1 = 0;
float theta1 = 0;
float volt1 = 0;
float duty = 0; 
float sumerror1 = 0;
float tau_d1 = 0;
float K = 0;
float D = 0;

// motor 2 
float current_d2 = 0;
float kp2 = 0;
float ki2 = 0;
float velocity2 = 0;
float current2 = 0;
float theta2 = 0;
float duty2 = 0; 
float volt2 = 0;
float sumerror2 = 0;
float tau_d2;
float K2 = 0;
float D2 = 0;

// general motor 
float R = 3.5;
float kb = 0.16;
float b = .00032;

// other parameters 
float desired_forearm = 0; 
float th1_i = 0; 
float the2_i = 0; 

Serial pc(USBTX, USBRX);    // USB Serial Terminal
ExperimentServer server;    // Object that lets us communicate with MATLAB
Timer t;                    // Timer to measure elapsed time of experiment
Ticker ControlLoop;         // Ticker to run current controller at high frequency

QEI encoderA(PE_9,PE_11, NC, 1200, QEI::X4_ENCODING);  // MOTOR A ENCODER (no index, 1200 counts/rev, Quadrature encoding)
QEI encoderB(PA_5, PB_3, NC, 1200, QEI::X4_ENCODING);  // MOTOR B ENCODER (no index, 1200 counts/rev, Quadrature encoding)

MotorShield motorShield(24000); // initialize the motor shield with a period of 12000 clock ticks or ~10kHZ

// function to calculate motor voltage according to current control law
void current_control() {
    // motor 1 
    float error1 = 0;
    theta1 = encoderA.getPulses()*(PULSE_TO_RAD)+th1_i;
    velocity1 = encoderA.getVelocity()*(PULSE_TO_RAD);
    current1 = -(motorShield.readCurrentA()*(30.0/65536.0)-15.0); //read current for motor A in amps. Note: this is a slightly different current sensor so its a different conversion than last lab.            
    error1 = current_d1 - current1;
    sumerror1 = sumerror1 + error1;

    // volt = 0; // EDIT THIS to use your current control law from Lab 2
    //voltage = kp*error + kd*(error-pasterror) + ki*sumerror;
    volt1 = R*current_d1 + kp*(error1) + ki*sumerror1 + kb*velocity1;

    // motor 2
    float error2 = 0;
    theta2 = encoderB.getPulses()*PULSE_TO_RAD+th2_i;
    velocity2 = encoderB.getVelocity()*PULSE_TO_RAD;
    current2 = -(motorShield.readCurrentB()*(30.0/65536.0)-15.0); //read current for motor A in amps. Note: this is a slightly different current sensor so its a different conversion than last lab.            
    error2 = current_d2 - current2;
    sumerror2 = sumerror2 + error2;

    volt2 = R*current_d2 + kp2*(error2) + ki2*sumerror2+ kb*velocity1;
   
   // motor within the duty 
    duty  = volt1/12.0;
    if (duty >  1) {
        duty =  1;
    }
    if (duty < -1) {
        duty = -1;  
    }

    if (duty >= 0){
        motorShield.motorAWrite(duty, 0);
    }
    else if (duty < 0){
        motorShield.motorAWrite(abs(duty), 1);
    }

    duty2  = volt2/12.0;
    if (duty2 >  1) {
        duty2 =  1;
    }
    if (duty2 < -1) {
        duty2 = -1;  
    }

    if (duty2 >= 0){
        motorShield.motorAWrite(duty, 0);
    }
    else if (duty2 < 0){
        motorShield.motorAWrite(abs(duty), 1);
    }
}

int main (void) {
    // Link the terminal with our server and start it up
    server.attachTerminal(pc);
    server.init();
   
    // Define array to hold input parameters from MATLAB
    float input_params[NUM_INPUTS];
    pc.printf("%f",input_params[0]);
   
    while(1) {
        // Run experiment every time input parameters are received from MATLAB
        if (server.getParams(input_params,NUM_INPUTS)) {
            // Unpack inputs
            // motor 1
            kp = input_params[0];
            ki = input_params[1];
            current_d1 = input_params[2];
            K = input_params[3];
            D = input_params[4];

            // motor 2
            kp2 = input_params[5];
            ki2 = input_params[6];
            current_d2 = input_params[7];
            K2 = input_params[8];
            D2 = input_params[9];

            // set up variables 
            desired_forearm = input_params[10];
            th1_i = input_params[11]; 
            th2_i = input_params[12];

            // Run current controller at 10kHz
            ControlLoop.attach(&current_control,0.0001);
           
            // Setup experiment
            t.reset();
            t.start();
            encoderA.reset();
            encoderB.reset();

            motorShield.motorAWrite(0, 0); //turn motor A off
           
            // Use the motor shield as follows:
            // motorShield.motorAWrite(DUTY CYCLE, DIRECTION), DIRECTION = 0 is forward, DIRECTION = 1 is backwards.
             
            // Run experiment
            while (t.read() < 5) {
                // Perform impedance control loop logic to calculate desired current
                // current_d = 0; // Set commanded current from impedance controller here.
                // motor 1
                tau_d1 = -K*theta1 - D*velocity1 + b * velocity1;

                // control limits 
                if (theta1 < -pi/6 || theta1 > pi/6){
                    tau_d1 = 0; // no torque if past limit 
                }

                // tau_d = -K*theta + b*velocity;
                current_d1 = tau_d1/kb; // Set commanded current from impedance controller here.

                // motor 2
                tau_d2 = -K*theta2 - D*velocity2 + b * velocity2;

                // control limits 
                if (theta2 < 0 || theta1 > desired_forearm + pi/3){
                    tau_d1 = 0; // no torque if past limit 
                }

                current_d2 = tau_d2/kb;
               
                // Send data to MATLAB
                float output_data[NUM_OUTPUTS];
                output_data[0] = t.read();
                output_data[1] = theta1;
                output_data[2] = velocity1;
                output_data[3] = current1;
                output_data[4] = volt1;
                output_data[5] = theta2;
                output_data[6] = velocity2;
                output_data[7] = current2;
                output_data[8] = volt2;

                server.sendData(output_data,NUM_OUTPUTS);              
                ThisThread::sleep_for(1); //run outer control loop at 1kHz
            }

            // Cleanup after each experiment
            ControlLoop.detach();
            server.setExperimentComplete();
            motorShield.motorAWrite(0, 0); //turn motor A off
        } // end if
    } // end while
   
} // end main