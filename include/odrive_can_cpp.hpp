#ifndef ODRIVE_CAN
#define ODRIVE_CAN

#include <stdlib.h>


#include <string>
#include <linux/can.h>
#include "odrive_enums.hpp"

namespace odrive_can{
    typedef struct 
    {
        AxisError axis_error;
        AxisState axis_state;
        MotorError motor_error;
        EncoderError encoder_error;
        ControllerError controller_error;
        bool trajectory_done;
    }heartbeat_t;

    typedef struct 
    {
        float pos_estimate;
        float vel_estimate;
    }encoder_estimate_t;

    typedef struct 
    {
        float shadow;
        float in_cpr;
    }encoder_count_t;

    typedef struct
    {
        float iq_setpoint;
        float iq_measured;
    }iq_t;

    typedef struct
    {
        float voltage;
        float current;
    }bus_power_t;
    
    /**
     * @class OdriveCan
     * @brief Interaction with an odrive axis communicating via CAN using can_socket
     */
    class OdriveCan
    {
    private:
        /* data */
        int can_socket;
    public:
        sockaddr_can address;
        can_frame last_frame;
        heartbeat_t last_heartbeat;
        ControlMode current_control_mode;

        OdriveCan(const std::string& interface, uint32_t filterId);
        ~OdriveCan();
        int send_message(const can_frame& frame);
        int receive_message(can_frame& frame);
        int e_stop();
        MotorError get_motor_error();
        EncoderError get_encoder_error();
        int set_axis_node_id(int new_address);
        int set_axis_requested_state(AxisState new_state);
        encoder_estimate_t get_encoder_estimates();
        encoder_count_t get_encoder_count();
        int set_controller_modes(int control_mode, int input_mode);
        int set_input_pos(float input_pos, int vel_ff, int toque_ff);
        int set_input_vel(float input_vel, float torque_ff);
        int set_input_toque(float input_torque);
        int set_limits(float vel_limit, float current_limit);
        int start_anticogging();
        int set_traj_vel_limit(float traj_vel_limit);
        int set_traj_accel_limit(float traj_accel_limit, float traj_decel_limit);
        int set_traj_inertia(float traj_inertia);
        int reboot();
        bus_power_t get_bus_voltage_and_current();
        int clear_errors();
        int set_position_gain(float pos_gain);
        int set_vel_gains(float vel_gain, float vel_integrator_gain);
        ControllerError get_controller_error();
    };

}


#endif
