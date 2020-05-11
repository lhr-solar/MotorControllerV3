#ifndef MAIN_H
#define MAIN_H

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
    #define CMD_DRIVE   (DC_BASE + DC_DRIVE)
    #define CMD_POWER   (DC_BASE + DC_POWER)
    #define CMD_RESET   (DC_BASE + DC_RESET)
    #define CMD_VEL     (MC_BASE + MC_VEL)

    // Other IDs
    // #define DC_BUS_CURRENT  0x900
    // #define DC_BUS_VOLTAGE  0x901
    // #define PHASE_B_CURRENT 0x902
    // #define PHASE_C_CURRENT 0x903
    // #define VEHICLE_VELOCITY 0x904
    // #define MOTOR_VELOCITY  0x905
    // #define VD 0x906
    // #define VQ 0x907
    // #define ID 0x908
    // #define IQ 0x909
    // #define BEMFD 0x90A
    // #define BEMFQ 0x90B
    // #define HEAT_SINK_TEMPERATURE 0x90C
    // #define MOTOR_TEMPERATURE 0x90D
    // #define DC_BUS_AMP_HOURS 0x90E
    // #define ODOMETER 0x90F

    #define DASHBOARD   0x584      // Dashboard ID
    #define BPS         0x100      // BPS ID
    #define MOTOR_DISABLE (BPS + 0xA)

    // Error IDs
    #define ERR_CAN_READ        0x1
    #define ERR_CAN_WRITE_DASH  0x2
    #define ERR_CAN_WRITE_MC    0x3
    #define ERR_BAD_STATE       0x4
    #define ERR_BUFF_OVERFLOW   0x5

    // parsing data to and from motor controller
    #define High32bits(x) ((x) & 0xFFFFFFFF00000000)
    #define Low32bits(x)  ((x) & 0x00000000FFFFFFFF)

    // TODO: profile acceleration pedal potentiometer.
    #define ACCEL_FLOOR     .5   // minimum proportion value for acceleration pedal to be considered engaged.
    #define MAX_VELOCITY    100 // unobtainable velocity (in m/s) to enable torque control mode.
    #define MAX_CURRENT     100 // 100% maximum acceleration force
    #define REGEN_PERCENTAGE    125       // TODO: determine this
    #define RATE_DRIVE_COMMAND  0.010   // the rate that sendMotorController() is sending updates to the Tritium
    #define RATE_SEND_VEL       0.100        // the rate that sendDashboard() is sending CAN messages to dashboard

    // Message format received from dashboard
    struct __attribute__((packed)) Message {
        uint8_t regenPercentage : 8;
        uint8_t : 6;
        bool enable : 1;
        bool set : 1;
    };

    // Message format sent to motor controller
    struct __attribute__((packed)) MotorMessage {
        uint32_t motorCurrent : 32;  // floats as defined for the tritiums
        uint32_t motorVelocity : 32;
    };

    // error message that halts the operation of the motor controller.
    struct Error {
        int errorID;            // error ID - see defines
        union {            // value to be sent
            float vehicleVel;           // can be either the vehicle velocity (sendDashboard)
            MotorMessage driveCommand;  // motor current and motor velocity (sendMotorController)
            unsigned int canID;         // or CAN ID (readDashboard, readMotorController)
            int invState;      // or state that the motor control fell into
        };
    };

#endif