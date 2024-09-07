#ifndef CYBER_GEAR_DRIVER_DEFS_H
#define CYBER_GEAR_DRIVER_DEFS_H

/*
 * Define the command ID for the Xiaomi CyberGear motor driver
 */
#define CMD_GET_ID                    0
#define CMD_MOTION_CTRL               1
#define CMD_RESPONSE_MOTION_STATE     2
#define CMD_ENABLE                    3
#define CMD_STOP                      4
#define CMD_SET_MECH_POSITION_TO_ZERO 6
#define CMD_CHANGE_CAN_ID             7
#define CMD_RAM_READ                  17
#define CMD_RAM_WRITE                 18
#define CMD_GET_MOTOR_FAIL            21

/*
 * Define the address for the Xiaomi CyberGear motor driver
 */
#define ADDR_RUN_MODE            0x7005  //
#define ADDR_IQ_REF              0x7006  // Ref 1
#define ADDR_SPEED_REF           0x700A  // Ref 2
#define ADDR_LIMIT_TORQUE        0x700B  // Limit 1
#define ADDR_CURRENT_KP          0x7010  // Gain 1
#define ADDR_CURRENT_KI          0x7011  // Gain 2
#define ADDR_CURRENT_FILTER_GAIN 0x7014  // Filter Gain
#define ADDR_POSITION_REF        0x7016  // Ref 3
#define ADDR_LIMIT_SPEED         0x7017  // Limit 2
#define ADDR_LIMIT_CURRENT       0x7018  // Limit 3
#define ADDR_MECH_POS            0x7019  // Read Only
#define ADDR_IQF                 0x701A  // Read Only
#define ADDR_MECH_VEL            0x701B  // Read Only
#define ADDR_VBUS                0x701C  // Read Only
#define ADDR_ROTATION            0x701D
#define ADDR_POSITION_KP         0x701E  // Gain 3
#define ADDR_SPEED_KP            0x701F  // Gain 4
#define ADDR_SPEED_KI            0x7020  // Gain 5

/*
 * Define the control mode for the Xiaomi CyberGear motor driver
 */
#define MODE_MOTION   0x00
#define MODE_POSITION 0x01
#define MODE_SPEED    0x02
#define MODE_CURRENT  0x03

/*
 * Define the motor thresholds for the Xiaomi CyberGear motor driver
 */

// Referance
// ADDR_POSITION_REF        0x7016
#define POS_MAX 12.5f
#define POS_MIN -12.5f
// ADDR_ROTATION            0x701D
#define ROTATION_MAX 2
#define ROTATION_MIN -2
// ADDR_SPEED_REF           0x700A
#define SPD_REF_MAX 30.0f
#define SPD_REF_MIN -30.0f
// ADDR_IQ_REF              0x7006
#define IQ_REF_MAX 23.0f
#define IQ_REF_MIN -23.0f

// Limit
// ADDR_LIMIT_SPEED         0x7017
#define VEL_MAX 30.0f
#define VEL_MIN -30.0f
// ADDR_LIMIT_TORQUE        0x700B
#define T_MAX 12.0f
#define T_MIN -12.0f
// ADDR_LIMIT_CURRENT       0x7018
#define IQ_MAX 27.0f
#define IQ_MIN -27.0f
// 0x2018
#define LIMIT_SPD_MAX 30.0f
#define LIMIT_SPD_MIN 0.0f
// 0x2007
#define LIMIT_TORQUE_MAX 0.0f
#define LIMIT_TORQUE_MIN 12.0f

// Gein
// MOTION_CTRL POSITION KP
#define MP_KP_MAX 200.0f
#define MP_KP_MIN 0.0f
// MOTION_CTRL VELOCITY KD
#define MV_KD_MAX 200.0f
#define MV_KD_MIN 0.0f
// ADDR_POSITION_KP         0x701E
#define POS_KP_MAX 200.0f
#define POS_KP_MIN 0.0f
// ADDR_SPEED_KP            0x701F
#define SPD_KP_MAX 200.0f
#define SPD_KP_MIN 0.0f
// ADDR_SPEED_KI            0x7020
#define SPD_KI_MAX 200.0f
#define SPD_KI_MIN 0.0f
// ADDR_CURRENT_KP          0x7010
// 0x2012
#define CUR_KP_MAX 200.0f
#define CUR_KP_MIN 0.0f
// ADDR_CURRENT_KI          0x7011
// 0x2013
#define CUR_KI_MAX 200.0f
#define CUR_KI_MIN 0.0f

// FILTER_GAIN
// ADDR_CURRENT_FILTER_GAIN 0x7014
#define CURRENT_FILTER_GAIN_MAX 1.0f
#define CURRENT_FILTER_GAIN_MIN 0.0f
// ADDR_SPEED_FILTER_GAIN   0x2014
#define SPEED_FILTER_GAIN_MAX 1.0f
#define SPEED_FILTER_GAIN_MIN 0.0f

/*
 * Define the default values for the Xiaomi CyberGear motor driver
 */
#define DEFAULT_CURRENT_KP          0.125f   // 0x7010
#define DEFAULT_CURRENT_KI          0.0158f  // 0x7011
#define DEFAULT_CURRENT_FINTER_GAIN 0.1f     // 0x7014
#define DEFAULT_POSITION_KP         30.0f    // 0x701E
#define DEFAULT_VELOCITY_KP         2.0f     // 0x701F
#define DEFAULT_VELOCITY_KI         0.002f   // 0x7020
#define DEFAULT_VELOCITY_LIMIT      2.0f     // 0x7017
#define DEFAULT_CURRENT_LIMIT       23.0f    // 0x7018
#define DEFAULT_TORQUE_LIMIT        12.0f    // 0x700B

#define RET_CYBERGEAR_OK             0x00
#define RET_CYBERGEAR_MSG_NOT_AVAIL  0x01
#define RET_CYBERGEAR_INVALID_CAN_ID 0x02
#define RET_CYBERGEAR_INVALID_PACKET 0x03

#define CW  1
#define CCW -1

#endif  // !CYBER_GEAR_DRIVER_DEFS_H
