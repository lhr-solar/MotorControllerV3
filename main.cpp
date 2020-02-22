/**
 * main.cpp
 * @authors:    Matthew Yu
 *              William Blount
 *              Ethan xxx
 *              Chase Block
 * @last_modified: 2/22/20
 * @description:
 *      Motor Control Board Program. This program operates the tritium controllers,
 *      as well as manage data over CAN with Dashboard.
 */

#include "mbed.h"

// CAN base address and offsets (Tritium)
#define DC_BASE     0x220   // Driver controls base address- reading da
#define MC_BASE     0x240   // Motor controls base address
// Control Command offsets
#define DC_DRIVE    0x01    // Offset for motor drive command
#define DC_POWER    0x02    // Offset for motor power command
#define DC_RESET    0x03    // Offset for reset command
#define DC_SWITCH   0x04    // Offset for phase current measurement
// Receive Data Commands offsets
#define MC_ID       0x00    // Device identifier message
#define MC_STATUS   0x01    // Status of motor (see WavesSculptor Communications Protocol TRI50.008 v5) 
#define MC_VEL      0x03    // Velocity measurement offset
// Commands
#define CMD_DRIVE   (DC_BASE + DC DRIVE)
#define CMD_POWER   (DC_BASE + DC_POWER)
#define CMD_RESET   (DC_BASE + DC_RESET)
#define CMD_VEL     (MC_BASE + MC_VEL)

// Other IDs
#define MAX_VELOCITY    100 // motor velocity in m/s
#define MAX_CURRENT     1.0  // desired motor current as percentage of max current

#define DC_BUS_CURRENT  0x900
#define DC_BUS_VOLTAGE  0x901
#define PHASE_B_CURRENT 0x902
#define PHASE_C_CURRENT 0x903
#define VEHICLE_VELOCITY 0x904
#define MOTOR_VELOCITY  0x905
#define VD 0x906
#define VQ 0x907
#define ID 0x908
#define IQ 0x909
#define BEMFD 0x90A
#define BEMFQ 0x90B
#define HEAT_SINK_TEMPERATURE 0x90C
#define MOTOR_TEMPERATURE 0x90D
#define DC_BUS_AMP_HOURS 0x90E
#define ODOMETER 0x90F

#define DASHBOARD  0x584      // Dashboard ID

// Error IDs
#define ERR_CAN_READ        0x1
#define ERR_CAN_WRITE_DASH  0x2
#define ERR_CAN_WRITE_MC    0x3

// parsing data to and from motor controller
#define High32bits(x) ((x) & 0xFFFF_FFFF_0000_0000)
#define Low32bits(x)  ((x) & 0x0000_0000_FFFF_FFFF)

// Message format received from dashboard
struct __attribute__((packed)) Message {
    uint8_t regenPercentage : 8;
    uint8_t : 6;
    bool enable : 1;
    bool set : 1;
};
// Message format sent to motor controller
struct __attribute__((packed)) MotorMessage {
    float highFloat : 32;
    float lowFloat : 32;
};

// error message that halts the operation of the motor controller.
struct Error {
    int errorID;            // error ID - see defines
    union data {            // value to be sent
        float vehicleVel;           // can be either the vehicle velocity (sendDashboard)
        MotorMessage driveCommand;  // motor current and motor velocity (sendMotorController)
        unsigned int canID;         // or CAN ID (readDashboard, readMotorController)
    };
};

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
Error errorFlag = (Error) {.errorID = 0, .data = 0};

// TODO: profile acceleration pedal potentiometer.
#define ACCEL_FLOOR     .5   // minimum proportion value for acceleration pedal to be considered engaged.
#define MAX_VELOCITY    100 // unobtainable velocity (in m/s) to enable torque control mode.
#define MAX_CURRENT     100 // 100% maximum acceleration force
#define REGEN_PERCENTAGE = 125;       // TODO: determine this
#define RATE_DRIVE_COMMAND = 0.100;   // the rate that sendMotorController() is sending updates to the Tritium
#define RATE_SEND_VEL = 0.100;        // the rate that sendDashboard() is sending CAN messages to dashboard


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
    errorFlag = (Error) {.errorID = 0, .data = 0};

    // acceleration pedal defaults as not pushed
    accelPot = 0;

    // set default drive state
    currState = futureState = DRIVE;

    while(errMsg->errorID != 0) { // until program receives an error
        // read accelPot
        accelPot = ain.read()   // note that the range is [0.0,1.0]
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
                state = DRIVE;
                // TODO: add watchdog - start counting messages - if x messages are missed, just start emergency protocols
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
            printf("Invalid message id: %x\n", errorFlag.data);
            break;
        case ERR_CAN_WRITE_DASH:
            printf("Failed to write to dashboard the vehicle speed: %f\n", errorFlag.data);
            break;
        case ERR_CAN_WRITE_MC:
            printf("Failed to write to motor controller the DRIVE COMMAND: %x\n", errorFlag.data);
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
 *      packed char in the form of 00000|Regen|Enable|Set
 */
void readDashboard() {
    while(1) {
        CANMessage msg;
        if(canCAR.read(msg)) {
            if(msg.id == DASHBOARD) {
                // getting a packed struct - see Message struct
                data = (Message *) msg.data;
            }else {
                errorFlag = (Error) {.errorID = ERR_CAN_READ, .data = msg.id};
            }
        }
    }
}

/**
 * sendDashboard attempts to send the following to dashboard and telemetry
 *      (float) vehicle velocity
 */
void sendDashboard() {
    if(!canCAR.write(CANMessage(MC_BASE + MC_VEL, vehicleVel))) {
        errorFlag = (Error) {.errorID = ERR_CAN_WRITE_DASH, .data = vehicleVel};
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
                vehicleVel = High32bits(msg.data) >> 32;    // vehicle velocity is higher 32 bits - shift right to grab it
            }else {
                errorFlag = (Error) {.errorID = ERR_CAN_READ, .data = msg.id};
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
    MotorMessage msg = (Low32bits(motorCurrent) << 32) + Low32bits(motorVelocity);
    if(!canMC.write(CANMessage(CMD_DRIVE), msg)) {
        errorFlag = (Error) {.errorID = ERR_CAN_WRITE_MC, .data = msg};
    }
}