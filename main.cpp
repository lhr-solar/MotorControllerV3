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
// DigitalOut LED8(PF_2);
// DigitalOut LED7(PA_7);
// DigitalOut LED6(PF_10);
// DigitalOut LED5(PF_5);
// DigitalOut LED4a(PF_3);
// DigitalOut LED3a(PC_3);
// DigitalOut LED2a(PC_0);
// DigitalOut LED1a(PA_3);

// function definitions
void readCarCAN();
void sendDashboard();
void readMotorController();
void sendMotorController();
int updateState();

void testMC();


// Object instances
// CAN lines
CAN canCar(PD_0, PD_1, 125000); // canCAR is car CAN  (Rx, Tx, speed)
CAN canMC(PB_5, PB_6, 100000);  // canMC is Tritium Motor Controller CAN  (Rx, Tx, speed)
// gas pedal potentiometer
AnalogIn ain(PB_0);
// communication line for PC - read from putty
Serial pc(USBTX, USBRX);
// Threads and interrupts
Thread t_readDashboard;
Thread t_readMC;
Ticker i_sendMC;
Ticker i_sendDashboard;
// Mutexes to protect writes 

// expecting an input 0-7 (state)
enum {DRIVE=0, CRUISE_ENABLE=2, CRUISE_SET=3, REGEN=4};
int currState;

// contains relevant state data obtained from the dashboard.
Message * data;
// internal program state of the vehicle
float vehicleVel = 0;
float motorCurrent = 0;
float motorVelocity = 0;
float accelPot = 0;
bool motorDisabled = True;
float targetVel = 0;

Error errorFlag = (Error) {
    .errorID = 0,
    .canID = 0
};

int main () {
    //startup sequence - initialize threads/interrupts
    t_readDashboard.start(callback(readCarCAN));
    t_readMC.start(callback(readMotorController));
    i_sendMC.attach(&sendMotorController, RATE_DRIVE_COMMAND);
    i_sendDashboard.attach(&sendDashboard, RATE_SEND_VEL);

    // set up internal testing thread
    Thread t_test;
    t_test.start(callback(testMC));

    // The car initially starts at 0 velocity.
    // received from the motor controller
    vehicleVel = 0;     

    // sent to the motor controller
    motorCurrent = 0;
    motorVelocity = 0;

    // default cruise control is off
    bool cruiseSet = false; 
    targetVel = 0;

    // set error flag to 0 (nominal), by default.
    errorFlag = (Error) {
        .errorID = 0,
        .canID = 0    
    };

    // acceleration pedal defaults as not pushed
    accelPot = 0;

    // set default drive state
    currState = DRIVE;

    // disable operation until BPS tells us to
    motorDisabled = True;

    while(errorFlag.errorID != 0) { // until program receives an error
        // update current state.
        currState = updateState(); 
        // stall until motor is enabled        
        while(motorDisabled){
            // we will define the behavior of the motor when it is disabled as at rest.
            // it overrides any commands from the dashboard until it is enabled.
            currState = DRIVE;
            motorCurrent = 0;
            motorVelocity = 0;
        };
        
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
            CRUISE_ENABLE: 
                // accelerate if ACCCEL is "ON"
                if(accelPot > ACCEL_FLOOR && !motorDisabled) {
                    motorVelocity = MAX_VELOCITY;
                    motorCurrent = accelPot;
                }else { // coast
                    // turn off motor current (acceleration force is 0)
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
                // reset cruise control to not jump back up vehicle speed to a set target velocity
                targetVel = 0; 
                break;
            default:
                pc.printf("Motor Control - Invalid state: %i\n", currState);
                errorFlag = (Error) {.errorID = ERR_BAD_STATE, .invState = currState};
        }

        // reset target velocity when out of cruise set mode.
        if(currState != CRUISE_SET) cruiseSet = false;    
    }
     
    switch (errorFlag.errorID) {
        case ERR_CAN_READ:
            pc.printf("Motor Control - Invalid message id: %x\n", errorFlag.canID);
            break;
        case ERR_CAN_WRITE_DASH:
            pc.printf("Motor Control - Failed to write to dashboard the vehicle speed: %f\n", errorFlag.vehicleVel);
            break;
        case ERR_CAN_WRITE_MC:
            pc.printf("Motor Control - Failed to write to motor controller the DRIVE COMMAND: %x\n", errorFlag.driveCommand);
            break;
        case ERR_BAD_STATE:
            pc.printf("Motor Control - Switch statement progressed to a bad state: %x\n", errorFlag.invState);
            break;
        case ERR_BUFF_OVERFLOW:
            pc.printf("Motor Control - Send methods encountered a buffer overflow: %f\n", errorFlag.vehicleVel);
            break;
        default:
            pc.printf("Motor Control - Undefined error: %i\n", errorFlag.errorID);
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
 *      we RECIEVE  (int) regenPercentage, (bool) cruise set, (bool) cruise enable from the Dashboard
 *      we also RECEIVE (bool) motorDisable from the BPS.
 * TODO: readCarCAN and readMotorController - setting variables - should be made atomic
 */

/**
 * readCarCAN attempts to read the following from the dashboard
 *      see Message struct
 * readCarCAN also checks for a motorDisable signal from BPS. This occurs when the BPS needs to go to a charging state.
 * NOTE: BPS motorDisable signal must received at least once every 5s.
 *      Consider the failure rate of the message: the message is sent 10 times every 5s, the message must fail to be sent 9 times before the controller halts the motor.
 */
void readCarCAN() {
    // TODO: add watchdog - start counting messages - if x messages are missed, just start emergency protocols
    while(1) {
        CANMessage msg;
        if(canCar.read(msg)) {
            if(msg.id == DASHBOARD) {
                // getting a packed struct - see Message struct
                data = (Message *) msg.data;
            }else if(msg.id == MOTOR_DISABLE) {
                // getting a boolean
                if(msg.data) {
                    motorDisabled = True;
                }else {
                    motorDisabled = False;
                }
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


/**
 * testMC sends a series of commands predefined by the following data structure. 
 * this independent thread acts in place of the external Dashboard and BPS, setting the state for important signals.
 */
struct State {
    int length;
    uint8_t regenPercentage;
    bool enable;
    bool set;
    bool motorDisabled;
}

const MAX_STATES = 10;
State[MAX_STATES] states = {
    {10, 0, 0, 0, 1},   // all off, motor disabled - state DRIVE. Try to drive motor. Should fail.
    {10, 0, 0, 0, 0},   // all off, motor  enabled - state DRIVE. Try to drive motor. Should work. 
    {10, 0, 0, 1, 0},   // cruise set on - state DRIVE(0). Try to drive motor. Should work.
    {10, 0, 1, 0, 0},   // cruise en on - state ENABLE(2). Try to drive motor. Should work.
    {10, 0, 1, 1, 0},   // cruise set, en on - state SET(3). Target velocity should be set similar to last data point's vehicle velocity. Try to drive motor and then coast. Should accelerate then stop at cruise floor
    {10, 10, 1, 1, 0},  // regen on - state SET(3).  regen < REGEN_PERCENTAGE (25)
    {10, 50, 0, 0, 0},  // regen on - state REGEN(4). motor velocity should drop to 0 over time.
    {10, 0, 0, 0, 0},   // state DRIVE. Try to drive motor. Should work.
    {10, 0, 0, 0, 1},   // all off, motor disabled- state DRIVE. Try to drive motor. Should fail.
    {10, 0, 0, 0, 1}
}

void testMC() {
    int counter = 0;
    while(counter < MAX_STATES) {
        // get at 2 data points for each state

        // print motor control status
        pc.printf("\nTest Line %i --------\n", counter);
        pc.printf(" State: %i\n
                    Vehicle Velocity: %f\n
                    Set Motor Current: %f\n
                    Set Motor Velocity: %f\n
                    Accel Pot Value: %f\n
                    ----------------------\n
                    Motor Disabled: %d\n
                    Regen Percentage: %f\n
                    Cruise Enable: %d\n
                    Cruise Set: %d\n
                    Target Velocity: %f\n
                    ----------------------\n
                    Error Flag: %i\n", 
                    currState, vehicleVel, motorCurrent, motorVelocity, accelPot, motorDisabled, data->regenPercentage, data->enable, data->set, targetVel, errorFlag.errorID);
        pc.printf("Test Line %i --------\n\n", counter);

        Thread::wait(states[counter]*500);

        // print motor control status
        pc.printf("\nTest Line %i --------\n", counter);
        pc.printf(" State: %i\n
                    Vehicle Velocity: %f\n
                    Set Motor Current: %f\n
                    Set Motor Velocity: %f\n
                    Accel Pot Value: %f\n
                    ----------------------\n
                    Motor Disabled: %d\n
                    Regen Percentage: %f\n
                    Cruise Enable: %d\n
                    Cruise Set: %d\n
                    Target Velocity: %f\n
                    ----------------------\n
                    Error Flag: %i\n", 
                    currState, vehicleVel, motorCurrent, motorVelocity, accelPot, motorDisabled, data->regenPercentage, data->enable, data->set, targetVel, errorFlag.errorID);
        pc.printf("Test Line %i --------\n\n", counter);

        // set status according to test regime
        data->regenPercentage = (float) states[counter].regenPercentage;
        data->enable = states[counter].enable;
        data->set = states[counter].set;
        motorDisabled = states[counter].motorDisabled;
        
        Thread::wait(states[counter]*500);
        // advance state
        counter++;
    }
}