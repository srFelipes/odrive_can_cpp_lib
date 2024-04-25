#ifndef ODRIVE_CAN
#define ODRIVE_CAN

#include <stdlib.h>
#include <thread>

#include <string>
#include <linux/can.h>
#include "odrive_enums.hpp"




namespace odrive_can{
    typedef struct 
    {
        uint32_t axis_error;
        AxisState axis_state;
        bool motor_error_flag;
        bool encoder_error_flag;
        bool controller_error_flag;
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
        int talk_socket;
        int listen_socket;
        
        int send_message(const can_frame& frame);
        int send_message(cmd_id command, bool is_rtr);
        int send_message(cmd_id command, unsigned char msg[], int msg_size);


        int receive_message(can_frame& frame);

        void listen_routine();

        

    public:
        const std::string this_interface;
        /**
         * @brief boolean variable used to signal when to kill the listening_thread
         * 
         */
        bool listening;
        /**
         * @brief the thread that is continuosly listening to the can_thread
         * 
         */
        std::thread listening_thread;

        /**
         * @brief Odrive axis address
         * 
         */
        sockaddr_can address;

        /**
         * @brief Last received frame
         * 
         */
        can_frame last_frame;

        /**
         * @brief last received heartbeat message
         * 
         */
        heartbeat_t last_heartbeat;

        /**
         * @brief current state and control mode of the axis
         * 
         */
        ControlMode current_control_mode;

        /**
         * @brief odrive axis id, the odrv arbitration id is axis_id<<5 | cmd_id
         * 
         */
        uint32_t axis_id;

        /**
         * @brief Construct a new Odrive Can object
         * 
         * @param interface String containing the name of the interface to use, vcan0 for test\
         *                   can0 for usage with the real device 
         * @param filterId Axis Id note that it should be  axis_id << 5
         */
        OdriveCan(const std::string& interface, uint32_t axis_id_param);

        /**
         * @brief Destroy the Odrive Can object
         * 
         */
        ~OdriveCan();

        

        /**
         * @brief sends an e_stop message, cmd_id = 0x002, the Odrive stops inmeditally
         * 
         * @return int 0 on success
         */
        int e_stop();

        /**
         * @brief Gets the motor error, in case of no error, return 0
         * 
         * @return MotorError an enum of the current motor error
         */
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
        int odrv_can_id(cmd_id cmd);
    };
}


#endif
