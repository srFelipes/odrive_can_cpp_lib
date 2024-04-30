
// TODO: This file is dangerous because the enums could potentially change between API versions. Should transmit as part of the JSON.
// To regenerate this file, nagivate to the top level of the ODrive repository and run:
// python Firmware/interface_generator_stub.py --definitions Firmware/odrive-interface.yaml --template tools/enums_template.j2 --output tools/odrive/enums.py



#define GPIO_MODE_DIGITAL                         0
#define GPIO_MODE_DIGITAL_PULL_UP                 1
#define GPIO_MODE_DIGITAL_PULL_DOWN               2
#define GPIO_MODE_ANALOG_IN                       3
#define GPIO_MODE_UART_A                          4
#define GPIO_MODE_UART_B                          5
#define GPIO_MODE_UART_C                          6
#define GPIO_MODE_CAN_A                           7
#define GPIO_MODE_I2C_A                           8
#define GPIO_MODE_SPI_A                           9
#define GPIO_MODE_PWM                             10
#define GPIO_MODE_ENC0                            11
#define GPIO_MODE_ENC1                            12
#define GPIO_MODE_ENC2                            13
#define GPIO_MODE_MECH_BRAKE                      14
#define GPIO_MODE_STATUS                          15

// ODrive.StreamProtocolType
#define STREAM_PROTOCOL_TYPE_FIBRE                0
#define STREAM_PROTOCOL_TYPE_ASCII                1
#define STREAM_PROTOCOL_TYPE_STDOUT               2
#define STREAM_PROTOCOL_TYPE_ASCII_AND_STDOUT     3

// ODrive.Can.Protocol
#define PROTOCOL_SIMPLE                           0x00000001

// ODrive.Axis.AxisState
#define AXIS_STATE_UNDEFINED                      0
#define AXIS_STATE_IDLE                           1
#define AXIS_STATE_STARTUP_SEQUENCE               2
#define AXIS_STATE_FULL_CALIBRATION_SEQUENCE      3
#define AXIS_STATE_MOTOR_CALIBRATION              4
#define AXIS_STATE_ENCODER_INDEX_SEARCH           6
#define AXIS_STATE_ENCODER_OFFSET_CALIBRATION     7
#define AXIS_STATE_CLOSED_LOOP_CONTROL            8
#define AXIS_STATE_LOCKIN_SPIN                    9
#define AXIS_STATE_ENCODER_DIR_FIND               10
#define AXIS_STATE_HOMING                         11
#define AXIS_STATE_ENCODER_HALL_POLARITY_CALIBRATION  12
#define AXIS_STATE_ENCODER_HALL_PHASE_CALIBRATION  13

// ODrive.Encoder.Mode
#define ENCODER_MODE_INCREMENTAL                  0
#define ENCODER_MODE_HALL                         1
#define ENCODER_MODE_SINCOS                       2
#define ENCODER_MODE_SPI_ABS_CUI                  256
#define ENCODER_MODE_SPI_ABS_AMS                  257
#define ENCODER_MODE_SPI_ABS_AEAT                 258
#define ENCODER_MODE_SPI_ABS_RLS                  259
#define ENCODER_MODE_SPI_ABS_MA732                260

// ODrive.Controller.ControlMode
#define CONTROL_MODE_VOLTAGE_CONTROL              0
#define CONTROL_MODE_TORQUE_CONTROL               1
#define CONTROL_MODE_VELOCITY_CONTROL             2
#define CONTROL_MODE_POSITION_CONTROL             3

// ODrive.Controller.InputMode
#define INPUT_MODE_INACTIVE                       0
#define INPUT_MODE_PASSTHROUGH                    1
#define INPUT_MODE_VEL_RAMP                       2
#define INPUT_MODE_POS_FILTER                     3
#define INPUT_MODE_MIX_CHANNELS                   4
#define INPUT_MODE_TRAP_TRAJ                      5
#define INPUT_MODE_TORQUE_RAMP                    6
#define INPUT_MODE_MIRROR                         7
#define INPUT_MODE_TUNING                         8

// ODrive.Motor.MotorType
#define MOTOR_TYPE_HIGH_CURRENT                   0
#define MOTOR_TYPE_GIMBAL                         2
#define MOTOR_TYPE_ACIM                           3

// ODrive.Error
#define ODRIVE_ERROR_NONE                         0x00000000
#define ODRIVE_ERROR_CONTROL_ITERATION_MISSED     0x00000001
#define ODRIVE_ERROR_DC_BUS_UNDER_VOLTAGE         0x00000002
#define ODRIVE_ERROR_DC_BUS_OVER_VOLTAGE          0x00000004
#define ODRIVE_ERROR_DC_BUS_OVER_REGEN_CURRENT    0x00000008
#define ODRIVE_ERROR_DC_BUS_OVER_CURRENT          0x00000010
#define ODRIVE_ERROR_BRAKE_DEADTIME_VIOLATION     0x00000020
#define ODRIVE_ERROR_BRAKE_DUTY_CYCLE_NAN         0x00000040
#define ODRIVE_ERROR_INVALID_BRAKE_RESISTANCE     0x00000080

// ODrive.Can.Error
#define CAN_ERROR_NONE                            0x00000000
#define CAN_ERROR_DUPLICATE_CAN_IDS               0x00000001

// ODrive.Axis.Error
#define AXIS_ERROR_NONE                           0x00000000
#define AXIS_ERROR_INVALID_STATE                  0x00000001
#define AXIS_ERROR_MOTOR_FAILED                   0x00000040
#define AXIS_ERROR_SENSORLESS_ESTIMATOR_FAILED    0x00000080
#define AXIS_ERROR_ENCODER_FAILED                 0x00000100
#define AXIS_ERROR_CONTROLLER_FAILED              0x00000200
#define AXIS_ERROR_WATCHDOG_TIMER_EXPIRED         0x00000800
#define AXIS_ERROR_MIN_ENDSTOP_PRESSED            0x00001000
#define AXIS_ERROR_MAX_ENDSTOP_PRESSED            0x00002000
#define AXIS_ERROR_ESTOP_REQUESTED                0x00004000
#define AXIS_ERROR_HOMING_WITHOUT_ENDSTOP         0x00020000
#define AXIS_ERROR_OVER_TEMP                      0x00040000
#define AXIS_ERROR_UNKNOWN_POSITION               0x00080000

// ODrive.Motor.Error
#define MOTOR_ERROR_NONE                          0x00000000
#define MOTOR_ERROR_PHASE_RESISTANCE_OUT_OF_RANGE  0x00000001
#define MOTOR_ERROR_PHASE_INDUCTANCE_OUT_OF_RANGE  0x00000002
#define MOTOR_ERROR_DRV_FAULT                     0x00000008
#define MOTOR_ERROR_CONTROL_DEADLINE_MISSED       0x00000010
#define MOTOR_ERROR_MODULATION_MAGNITUDE          0x00000080
#define MOTOR_ERROR_CURRENT_SENSE_SATURATION      0x00000400
#define MOTOR_ERROR_CURRENT_LIMIT_VIOLATION       0x00001000
#define MOTOR_ERROR_MODULATION_IS_NAN             0x00010000
#define MOTOR_ERROR_MOTOR_THERMISTOR_OVER_TEMP    0x00020000
#define MOTOR_ERROR_FET_THERMISTOR_OVER_TEMP      0x00040000
#define MOTOR_ERROR_TIMER_UPDATE_MISSED           0x00080000
#define MOTOR_ERROR_CURRENT_MEASUREMENT_UNAVAILABLE  0x00100000
#define MOTOR_ERROR_CONTROLLER_FAILED             0x00200000
#define MOTOR_ERROR_I_BUS_OUT_OF_RANGE            0x00400000
#define MOTOR_ERROR_BRAKE_RESISTOR_DISARMED       0x00800000
#define MOTOR_ERROR_SYSTEM_LEVEL                  0x01000000
#define MOTOR_ERROR_BAD_TIMING                    0x02000000
#define MOTOR_ERROR_UNKNOWN_PHASE_ESTIMATE        0x04000000
#define MOTOR_ERROR_UNKNOWN_PHASE_VEL             0x08000000
#define MOTOR_ERROR_UNKNOWN_TORQUE                0x10000000
#define MOTOR_ERROR_UNKNOWN_CURRENT_COMMAND       0x20000000
#define MOTOR_ERROR_UNKNOWN_CURRENT_MEASUREMENT   0x40000000
#define MOTOR_ERROR_UNKNOWN_VBUS_VOLTAGE          0x80000000
#define MOTOR_ERROR_UNKNOWN_VOLTAGE_COMMAND       0x100000000
#define MOTOR_ERROR_UNKNOWN_GAINS                 0x200000000
#define MOTOR_ERROR_CONTROLLER_INITIALIZING       0x400000000
#define MOTOR_ERROR_UNBALANCED_PHASES             0x800000000

// ODrive.Controller.Error
#define CONTROLLER_ERROR_NONE                     0x00000000
#define CONTROLLER_ERROR_OVERSPEED                0x00000001
#define CONTROLLER_ERROR_INVALID_INPUT_MODE       0x00000002
#define CONTROLLER_ERROR_UNSTABLE_GAIN            0x00000004
#define CONTROLLER_ERROR_INVALID_MIRROR_AXIS      0x00000008
#define CONTROLLER_ERROR_INVALID_LOAD_ENCODER     0x00000010
#define CONTROLLER_ERROR_INVALID_ESTIMATE         0x00000020
#define CONTROLLER_ERROR_INVALID_CIRCULAR_RANGE   0x00000040
#define CONTROLLER_ERROR_SPINOUT_DETECTED         0x00000080

// ODrive.Encoder.Error
#define ENCODER_ERROR_NONE                        0x00000000
#define ENCODER_ERROR_UNSTABLE_GAIN               0x00000001
#define ENCODER_ERROR_CPR_POLEPAIRS_MISMATCH      0x00000002
#define ENCODER_ERROR_NO_RESPONSE                 0x00000004
#define ENCODER_ERROR_UNSUPPORTED_ENCODER_MODE    0x00000008
#define ENCODER_ERROR_ILLEGAL_HALL_STATE          0x00000010
#define ENCODER_ERROR_INDEX_NOT_FOUND_YET         0x00000020
#define ENCODER_ERROR_ABS_SPI_TIMEOUT             0x00000040
#define ENCODER_ERROR_ABS_SPI_COM_FAIL            0x00000080
#define ENCODER_ERROR_ABS_SPI_NOT_READY           0x00000100
#define ENCODER_ERROR_HALL_NOT_CALIBRATED_YET     0x00000200

// ODrive.SensorlessEstimator.Error
#define SENSORLESS_ESTIMATOR_ERROR_NONE           0x00000000
#define SENSORLESS_ESTIMATOR_ERROR_UNSTABLE_GAIN  0x00000001
#define SENSORLESS_ESTIMATOR_ERROR_UNKNOWN_CURRENT_MEASUREMENT  0x00000002

enum class AxisState{
    UNDEFINED                                = 0,
    IDLE                                     = 1,
    STARTUP_SEQUENCE                         = 2,
    FULL_CALIBRATION_SEQUENCE                = 3,
    MOTOR_CALIBRATION                        = 4,
    ENCODER_INDEX_SEARCH                     = 6,
    ENCODER_OFFSET_CALIBRATION               = 7,
    CLOSED_LOOP_CONTROL                      = 8,
    LOCKIN_SPIN                              = 9,
    ENCODER_DIR_FIND                         = 10,
    HOMING                                   = 11,
    ENCODER_HALL_POLARITY_CALIBRATION        = 12,
    ENCODER_HALL_PHASE_CALIBRATION           = 13
};

enum class EncoderMode{
    INCREMENTAL                              = 0,
    HALL                                     = 1,
    SINCOS                                   = 2,
    SPI_ABS_CUI                              = 256,
    SPI_ABS_AMS                              = 257,
    SPI_ABS_AEAT                             = 258,
    SPI_ABS_RLS                              = 259,
    SPI_ABS_MA732                            = 260,
};

enum class ControlMode{
    VOLTAGE_CONTROL                          = 0,
    TORQUE_CONTROL                           = 1,
    VELOCITY_CONTROL                         = 2,
    POSITION_CONTROL                         = 3,
};

enum class InputMode{
    INACTIVE                                 = 0,
    PASSTHROUGH                              = 1,
    VEL_RAMP                                 = 2,
    POS_FILTER                               = 3,
    MIX_CHANNELS                             = 4,
    TRAP_TRAJ                                = 5,
    TORQUE_RAMP                              = 6,
    MIRROR                                   = 7,
    TUNING                                   = 8,
};
enum class MotorType{
    HIGH_CURRENT                             = 0,
    GIMBAL                                   = 2,
    ACIM                                     = 3,
};

enum class ODriveError{
    NONE                                     = 0x00000000,
    CONTROL_ITERATION_MISSED                 = 0x00000001,
    DC_BUS_UNDER_VOLTAGE                     = 0x00000002,
    DC_BUS_OVER_VOLTAGE                      = 0x00000004,
    DC_BUS_OVER_REGEN_CURRENT                = 0x00000008,
    DC_BUS_OVER_CURRENT                      = 0x00000010,
    BRAKE_DEADTIME_VIOLATION                 = 0x00000020,
    BRAKE_DUTY_CYCLE_NAN                     = 0x00000040,
    INVALID_BRAKE_RESISTANCE                 = 0x00000080
};

enum class CanError{
    NONE                                     = 0x00000000,
    DUPLICATE_CAN_IDS                        = 0x00000001,
};

enum class AxisError{
    NONE                                     = 0x00000000,
    INVALID_STATE                            = 0x00000001,
    MOTOR_FAILED                             = 0x00000040,
    SENSORLESS_ESTIMATOR_FAILED              = 0x00000080,
    ENCODER_FAILED                           = 0x00000100,
    CONTROLLER_FAILED                        = 0x00000200,
    WATCHDOG_TIMER_EXPIRED                   = 0x00000800,
    MIN_ENDSTOP_PRESSED                      = 0x00001000,
    MAX_ENDSTOP_PRESSED                      = 0x00002000,
    ESTOP_REQUESTED                          = 0x00004000,
    HOMING_WITHOUT_ENDSTOP                   = 0x00020000,
    OVER_TEMP                                = 0x00040000,
    UNKNOWN_POSITION                         = 0x00080000,
};
enum class MotorError: unsigned long{
    NONE                                     = 0x00000000,
    PHASE_RESISTANCE_OUT_OF_RANGE            = 0x00000001,
    PHASE_INDUCTANCE_OUT_OF_RANGE            = 0x00000002,
    DRV_FAULT                                = 0x00000008,
    CONTROL_DEADLINE_MISSED                  = 0x00000010,
    MODULATION_MAGNITUDE                     = 0x00000080,
    CURRENT_SENSE_SATURATION                 = 0x00000400,
    CURRENT_LIMIT_VIOLATION                  = 0x00001000,
    MODULATION_IS_NAN                        = 0x00010000,
    MOTOR_THERMISTOR_OVER_TEMP               = 0x00020000,
    FET_THERMISTOR_OVER_TEMP                 = 0x00040000,
    TIMER_UPDATE_MISSED                      = 0x00080000,
    CURRENT_MEASUREMENT_UNAVAILABLE          = 0x00100000,
    CONTROLLER_FAILED                        = 0x00200000,
    I_BUS_OUT_OF_RANGE                       = 0x00400000,
    BRAKE_RESISTOR_DISARMED                  = 0x00800000,
    SYSTEM_LEVEL                             = 0x01000000,
    BAD_TIMING                               = 0x02000000,
    UNKNOWN_PHASE_ESTIMATE                   = 0x04000000,
    UNKNOWN_PHASE_VEL                        = 0x08000000,
    UNKNOWN_TORQUE                           = 0x10000000,
    UNKNOWN_CURRENT_COMMAND                  = 0x20000000,
    UNKNOWN_CURRENT_MEASUREMENT              = 0x40000000,
    UNKNOWN_VBUS_VOLTAGE                     = 0x80000000,
    UNKNOWN_VOLTAGE_COMMAND                  = 0x100000000,
    UNKNOWN_GAINS                            = 0x200000000,
    CONTROLLER_INITIALIZING                  = 0x400000000,
    UNBALANCED_PHASES                        = 0x800000000
};
enum class ControllerError{
    NONE                                     = 0x00000000,
    OVERSPEED                                = 0x00000001,
    INVALID_INPUT_MODE                       = 0x00000002,
    UNSTABLE_GAIN                            = 0x00000004,
    INVALID_MIRROR_AXIS                      = 0x00000008,
    INVALID_LOAD_ENCODER                     = 0x00000010,
    INVALID_ESTIMATE                         = 0x00000020,
    INVALID_CIRCULAR_RANGE                   = 0x00000040,
    SPINOUT_DETECTED                         = 0x00000080
};
enum class EncoderError{
    NONE                                     = 0x00000000,
    UNSTABLE_GAIN                            = 0x00000001,
    CPR_POLEPAIRS_MISMATCH                   = 0x00000002,
    NO_RESPONSE                              = 0x00000004,
    UNSUPPORTED_ENCODER_MODE                 = 0x00000008,
    ILLEGAL_HALL_STATE                       = 0x00000010,
    INDEX_NOT_FOUND_YET                      = 0x00000020,
    ABS_SPI_TIMEOUT                          = 0x00000040,
    ABS_SPI_COM_FAIL                         = 0x00000080,
    ABS_SPI_NOT_READY                        = 0x00000100,
    HALL_NOT_CALIBRATED_YET                  = 0x00000200
};
enum class SensorlessEstimatorError{
    NONE                                     = 0x00000000,
    UNSTABLE_GAIN                            = 0x00000001,
    UNKNOWN_CURRENT_MEASUREMENT              = 0x00000002
};

enum class cmd_id{
    CAN_OPEN_NMT_MSG         = 0x000,
    HEARTBEAT                = 0x001,
    ESTOP                    = 0x002,
    MOTOR_ERROR              = 0x003,
    ENCODER_ERROR            = 0x004,
    SENSORLESS_ERROR         = 0x005,
    NODE_ID                  = 0x006,
    SET_REQUESTED_STATE      = 0x007,
    SET_STARTUP_CONFIG       = 0x008,
    GET_ENCODER_ESTIMATES    = 0x009,
    GET_ENCODER_COUNT        = 0x00A,
    SET_CONTROLLER_MODES     = 0x00B,
    SET_INPUT_POS            = 0x00C,
    SET_INPUT_VEL            = 0x00D,
    SET_INPUT_TORQUE         = 0x00E,
    SET_LIMITS               = 0x00F,
    START_ANTICOGGING        = 0x010,
    SET_TRAJ_VEL_LIMIT       = 0x011,
    SET_TRAJ_ACCEL_LIMIT     = 0x012,
    SET_TRAJ_INERTIA         = 0x013,
    GET_IQ                   = 0x014,
    GET_SENSORLESS_ESTIMATES = 0x015,
    REBOOT_ODRV              = 0x016,
    GET_BUS_VOLT_AND_CURRENT = 0x017,
    CLEAR_ERRORS             = 0x018,
    SET_LINEAR_COUNT         = 0x019,
    SET_POS_GAIN             = 0x01A,
    SET_VEL_GAIN             = 0x01B,
    GET_ADC_VOLTAGE          = 0x01C,
    GET_CONTROLLER_ERROR     = 0x01D,
    CAN_OPEN_HEARTBEAT       = 0x700
};

#define MOTOR_ERROR_LEN 28

static uint64_t all_the_motor_errors[] = {
    static_cast<uint64_t>(MotorError::NONE),
    static_cast<uint64_t>(MotorError::PHASE_RESISTANCE_OUT_OF_RANGE),
    static_cast<uint64_t>(MotorError::PHASE_INDUCTANCE_OUT_OF_RANGE),
    static_cast<uint64_t>(MotorError::DRV_FAULT),
    static_cast<uint64_t>(MotorError::CONTROL_DEADLINE_MISSED),
    static_cast<uint64_t>(MotorError::MODULATION_MAGNITUDE),
    static_cast<uint64_t>(MotorError::CURRENT_SENSE_SATURATION),
    static_cast<uint64_t>(MotorError::CURRENT_LIMIT_VIOLATION),
    static_cast<uint64_t>(MotorError::MODULATION_IS_NAN),
    static_cast<uint64_t>(MotorError::MOTOR_THERMISTOR_OVER_TEMP),
    static_cast<uint64_t>(MotorError::FET_THERMISTOR_OVER_TEMP),
    static_cast<uint64_t>(MotorError::TIMER_UPDATE_MISSED),
    static_cast<uint64_t>(MotorError::CURRENT_MEASUREMENT_UNAVAILABLE),
    static_cast<uint64_t>(MotorError::CONTROLLER_FAILED),
    static_cast<uint64_t>(MotorError::I_BUS_OUT_OF_RANGE),
    static_cast<uint64_t>(MotorError::BRAKE_RESISTOR_DISARMED),
    static_cast<uint64_t>(MotorError::SYSTEM_LEVEL),
    static_cast<uint64_t>(MotorError::BAD_TIMING),
    static_cast<uint64_t>(MotorError::UNKNOWN_PHASE_ESTIMATE),
    static_cast<uint64_t>(MotorError::UNKNOWN_PHASE_VEL),
    static_cast<uint64_t>(MotorError::UNKNOWN_TORQUE),
    static_cast<uint64_t>(MotorError::UNKNOWN_CURRENT_COMMAND),
    static_cast<uint64_t>(MotorError::UNKNOWN_CURRENT_MEASUREMENT),
    static_cast<uint64_t>(MotorError::UNKNOWN_VBUS_VOLTAGE),
    static_cast<uint64_t>(MotorError::UNKNOWN_VOLTAGE_COMMAND),
    static_cast<uint64_t>(MotorError::UNKNOWN_GAINS),
    static_cast<uint64_t>(MotorError::CONTROLLER_INITIALIZING),
    static_cast<uint64_t>(MotorError::UNBALANCED_PHASES)
};

// Custom exception class for unexpected messages
class UnexpectedMessageException : public std::exception {
public:
    UnexpectedMessageException(const std::string& message)
        : m_message(message) {}

    // Override what() method to provide error message
    const char* what() const noexcept override {
        return m_message.c_str();
    }

private:
    std::string m_message;
};