/**
 * main.cpp
 * @authors:    Matthew Yu
 *              William Blount
 *              Ethan xxx
 *              Chase Block
 * @last_modified: 2/29/20
 * @description:
 *      Motor Control Board Program. This program operates the tritium controllers,
 *      as well as manage data over CAN with Dashboard.
 */

#include "mbed.h"
#include "main.h"

// TODO: determine whether these are necessary
// Serial pc(USBTX, USBRX);
// DigitalOut LED8(PF_2);
// DigitalOut LED7(PA_7);
// DigitalOut LED6(PF_10);
// DigitalOut LED5(PF_5);
// DigitalOut LED4a(PF_3);
// DigitalOut LED3a(PC_3);
// DigitalOut LED2a(PC_0);
// DigitalOut LED1a(PA_3);

// function definitions
void readDashboard();
void sendDashboard();
void readMotorController();
void sendMotorController();
int updateState();

// Object instances
CAN canCar(PD_0, PD_1, 125000); // canCAR is car CAN  (Rx, Tx, speed)
CAN canMC(PB_5, PB_6, 100000);  // canMC is Tritium Motor Controller CAN  (Rx, Tx, speed)
AnalogIn ain(PB_0);
// Threads and interrupts
Thread t_readDashboard;
Thread t_readMC;
Ticker i_sendMC;
Ticker i_sendDashboard;
// Mutexes to protect writes 

// expecting an input 0-7 (state)
enum {DRIVE=0, CRUISE_ENABLE=2, CRUISE_SET=3, REGEN=4};
int currState, futureState;

// contains relevant state data obtained from the dashboard.
Message * data;
// internal program state of the vehicle
float vehicleVel = 0;
float busCurrent = 0;
float motorCurrent = 0;
float motorVelocity = 0;
float accelPot = 0;
Error errorFlag = (Error) {
    .errorID = 0,
    .canID = 0
};

int main () {
    //startup sequence - initialize threads/interrupts
    t_readDashboard.start(callback(readDashboard));
    t_readMC.start(callback(readMotorController));
    i_sendMC.attach(&sendMotorController, RATE_DRIVE_COMMAND);
    i_sendDashboard.attach(&sendDashboard, RATE_SEND_VEL);

    // The car initially starts at 0 velocity.
    // received from the motor controller
    vehicleVel = 0;     

    // sent to the motor controller
    motorCurrent = 0;
    motorVelocity = 0;

    // default cruise control is off
    bool cruiseSet = false; 
    int targetVel = 0;

    // set error flag to 0 (nominal), by default.
    errorFlag = (Error) {
        .errorID = 0,
        .canID = 0    
    };

    // acceleration pedal defaults as not pushed
    accelPot = 0;

    // set default drive state
    currState = futureState = DRIVE;

    while(errorFlag.errorID != 0) { // until program receives an error
        // read accelPot
        accelPot = ain.read();   // note that the range is [0.0,1.0]
        /**
         * Function Table
         * MODE             | ACCEL     | FUNCTION
         * DRIVE/CRU_EN     | OFF       | COAST         // assumption is that regen = 0 (off)
         * DRIVE/CRU_EN     | ON        | ACCELERATE    // grab data from accel. pot
         * CRU_SET          | OFF       | HOLD SPEED
         * CRU_SET          | ON        | ACCELERATE
         * REGEN            | OFF/ON    | DECELERATE    // drop to target speed (a proportion of the regen pot data)
         */
        switch(currState) {
            DRIVE: 
                // accelerate if ACCCEL is "ON"
                if(accelPot > ACCEL_FLOOR) {
                    motorVelocity = MAX_VELOCITY;
                    motorCurrent = accelPot;
                }else { // coast
                    // turn off motor current (acceleration force is 0)
                    motorCurrent = 0;
                }
                break;
            CRUISE_ENABLE: 
                // accelerate if ACCCEL is "ON"
                if(accelPot > ACCEL_FLOOR) {
                    motorVelocity = MAX_VELOCITY;
                    motorCurrent = accelPot;
                }else { // coast
                    motorCurrent = 0;
                }
                break;
            CRUISE_SET: 
                // When in mode CRUISE_SET for the first time, we want to capture the current velocity and save it.
                if(!cruiseSet) { 
                    cruiseSet = true;
                    targetVel = vehicleVel;
                }

                // accelerate if ACCCEL is "ON"
                if(accelPot > ACCEL_FLOOR) {
                    motorVelocity = MAX_VELOCITY;
                    motorCurrent = accelPot;
                }else { // hold speed
                    motorVelocity = targetVel;
                    motorCurrent = MAX_CURRENT;
                }
                break;
            REGEN: 
                // TODO: test dropping velocity over time
                motorCurrent = motorCurrent*.9; // TODO: come up with more mathematical way to drop acceleration over time
                motorVelocity = 0;
                break;
            default:
                printf("Invalid state: %i\n", currState);
                errorFlag = (Error) {.errorID = ERR_BAD_STATE, .invState = currState};
        }

        // reset target velocity when out of cruise set mode.
        if(currState != CRUISE_SET) cruiseSet = false; 
        // update future state.
        futureState = updateState();
        // update current state.
        currState = futureState;    
    }
     
    switch (errorFlag.errorID) {
        case ERR_CAN_READ:
            printf("Invalid message id: %x\n", errorFlag.canID);
            break;
        case ERR_CAN_WRITE_DASH:
            printf("Failed to write to dashboard the vehicle speed: %f\n", errorFlag.vehicleVel);
            break;
        case ERR_CAN_WRITE_MC:
            printf("Failed to write to motor controller the DRIVE COMMAND: %x\n", errorFlag.driveCommand);
            break;
        case ERR_BAD_STATE:
            printf("Switch statement progressed to a bad state: %x\n", errorFlag.invState);
            break;
        case ERR_BUFF_OVERFLOW:
            printf("Send methods encountered a buffer overflow: %f\n", errorFlag.vehicleVel);
            break;
        default:
            printf("Undefined error: %i\n", errorFlag.errorID);
            break;
    }

    return errorFlag.errorID;
}

// updates the future state given data obtained from the dashboard.
int updateState() {
    // go in order of priority (regen, enable, set)
    int state = DRIVE;
    if(data->regenPercentage > REGEN_PERCENTAGE) {  // regen can be considered on
        state = REGEN;
    }else if(data->enable) {    // enable is on
        if(data->set) { // set is on
            state = CRUISE_SET; 
        }else { // set is off
            state = CRUISE_ENABLE;
        }
    }else { // drive - if set is on, we don't consider it since enable is off
        state = DRIVE;
    }
    return state;
}

/**
 * two CAN lines -  CAN line to tritium motor controllers (CAN2), 
 *                  CAN line to the main system           (CAN1)
 * motorCAN has two operations:
 *      we SEND     (float) current and (float) velocity
 *      we RECIEVE  (float) speed
 * carCAN has 
 *      we SEND     (float) speed [possibly other telemetry data]
 *      we RECIEVE  (int) regenPercentage, (bool) cruise set, (bool) cruise enable
 * TODO: readDashboard and readMotorController - setting variables - should be made atomic
 */

/**
 * readDashboard attempts to read the following from the dashboard
 *      see Message struct
 */
void readDashboard() {
    // TODO: add watchdog - start counting messages - if x messages are missed, just start emergency protocols
    while(1) {
        CANMessage msg;
        if(canCar.read(msg)) {
            if(msg.id == DASHBOARD) {
                // getting a packed struct - see Message struct
                data = (Message *) msg.data;
            }else {
                errorFlag = (Error) {.errorID = ERR_CAN_READ, .canID = msg.id};
            }
        }
    }
}

/**
 * sendDashboard attempts to send the following to dashboard and telemetry
 *      (float) vehicle velocity
 */
void sendDashboard() {
    // convert float to bytes
    char buffer[4];
    int ret = snprintf(buffer, sizeof buffer, "%f", vehicleVel);
    if(ret < 0) {
        errorFlag = (Error) {.errorID = ERR_CAN_WRITE_DASH, .vehicleVel = vehicleVel};    
    }else {
        if(!canCar.write(CANMessage(CMD_VEL, buffer, 4))) {
            errorFlag = (Error) {.errorID = ERR_BUFF_OVERFLOW, .vehicleVel = vehicleVel};
        }    
    }
}

/**
 * readMotorController attempts to read the following from the motor controller CAN
 *      (float) vehicle velocity
 */
void readMotorController() {
    while(1) {
        CANMessage msg;
        if(canMC.read(msg)) {
            if(msg.id == (CMD_VEL)) {                       // grabbing vehicle velocity determined by motor controller
                // grab only the upper 32 bits of the msg - vehicle velocity
                char data[4];
                data[0] = msg.data[7];
                data[1] = msg.data[6];
                data[2] = msg.data[5];
                data[3] = msg.data[4]; 
                vehicleVel = *(float*)&data;
            }else {
                errorFlag = (Error) {.errorID = ERR_CAN_READ, .canID= msg.id};
            }
        }
    }
}

/**
 * sendMotorController attempts to send the following to the tritium motor controller
 *      Motor Drive Command: [float - motor current][float - motor velocity]
 * NOTE: Motor Drive Command must be sent at least once every 250ms. 
 *      Consider the failure rate of the message: if the message is sent 10 times every 250ms, the message must fail to be sent 9 times before the controller halts the motor. 
 */
void sendMotorController() {
    // convert floats to MotorMessage
    uint32_t motorCurrBits = *((uint32_t*)&motorCurrent);
    uint32_t motorVelBits = *((uint32_t*)&motorVelocity);
    MotorMessage mtrMsg = {motorCurrBits, motorVelBits};
    // convert MotorMessage to char*
    CANMessage canMsg = CANMessage(CMD_DRIVE, (char*)&mtrMsg);
    if(!canMC.write(canMsg)) {
        errorFlag = (Error) {.errorID = ERR_CAN_WRITE_MC, .driveCommand = mtrMsg};
    }
}