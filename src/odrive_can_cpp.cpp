#include "odrive_can_cpp.hpp"
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <net/if.h>
#include <unistd.h>
#include <cstring>
#include <stdexcept>
#include <linux/can/raw.h>
#include <string>
#include <iostream>
#include <poll.h>

#define ODRV_CAN_MASK 0x7e0
#define ODRV_CMD_MASK 0b11111

namespace odrive_can{

int create_socket(const std::string& interface){
        int output_socket;
        output_socket = socket(PF_CAN, SOCK_RAW, CAN_RAW);
        if (output_socket == -1) {
            throw std::runtime_error("Failed to create socket");
        }
        // Set the interface name
        struct ifreq ifr;
        std::strncpy(ifr.ifr_name, interface.c_str(), IFNAMSIZ);
        ioctl(output_socket, SIOCGIFINDEX, &ifr);

        // Bind the socket to the CAN interface
        struct sockaddr_can addr;
        addr.can_family = AF_CAN;
        addr.can_ifindex = ifr.ifr_ifindex;
        if (bind(output_socket, reinterpret_cast<struct sockaddr*>(&addr), sizeof(addr)) == -1) {
            close(output_socket);
            throw std::runtime_error("Failed to bind socket to interface");
        }
        return output_socket;
    }

OdriveCan::OdriveCan(const std::string& interface, uint32_t axis_id_param)
: this_interface(interface), axis_id(axis_id_param)
{   listen_socket = create_socket(interface);
    talk_socket = create_socket(interface);
    listening = true;
    // Set filter (accept only specific CAN IDs)
    can_filter filter[1];
    filter[0].can_id = axis_id_param<<5;
    filter[0].can_mask = ODRV_CAN_MASK;

    setsockopt(listen_socket, SOL_CAN_RAW, CAN_RAW_FILTER, &filter, sizeof(filter));
    axis_id = axis_id_param;
    listening_thread = std::thread(&OdriveCan::listen_routine,this);
}
void OdriveCan::listen_routine(){
    int thread_soc = create_socket(this_interface);
    struct pollfd pollfds[1];
    pollfds[0].fd = thread_soc; // Set the file descriptor to monitor
    pollfds[0].events = POLLIN; // Set the events to monitor for (in this case, readability)

    can_filter filter[1];
    filter[0].can_id = axis_id<<5;
    filter[0].can_mask = ODRV_CAN_MASK;
    setsockopt(thread_soc, SOL_CAN_RAW, CAN_RAW_FILTER, &filter, sizeof(filter));

    while (listening){
        int ret = poll(pollfds, 1, 1); // Monitor indefinitely for events on the file descriptor
        if (ret > 0) {
            if (pollfds[0].revents & POLLIN) { // Check if the file descriptor is ready for reading
                // Read data from the socket
                ssize_t bytes_read = read(thread_soc, &last_frame, sizeof(can_frame));
                if (bytes_read < 0) {
                    // Handle error
                } else {
                    uint16_t cmd = last_frame.can_id&ODRV_CMD_MASK;
                    cmd += 0;
                    if (static_cast<uint16_t>(cmd_id::HEARTBEAT) == cmd){
                        memcpy(&last_heartbeat.axis_state,&last_frame.data[4],1);
                        last_heartbeat.controller_error_flag = last_frame.data[7]&1;
                        last_heartbeat.motor_error_flag = last_frame.data[5];
                        last_heartbeat.encoder_error_flag = last_frame.data[6];
                        last_heartbeat.trajectory_done = last_frame.data[7]>>7;
                    }
                }
            }
        } else if (ret < 0) {
            // Handle poll error
        }
    }
    return;
}

OdriveCan::~OdriveCan()
{   
    if (listening_thread.joinable()) {
            listening_thread.join();
        }
    close(listen_socket);
    close(talk_socket);
}
int OdriveCan::send_message(const can_frame& frame) {
    return write(talk_socket, &frame, sizeof(frame));
}
int OdriveCan::send_message(cmd_id command, unsigned char msg[], int msg_size){
    return 0;
}

int OdriveCan::send_message(cmd_id command, bool is_rtr){
    can_frame frame;
    frame.can_id = odrv_can_id(command);
    if (is_rtr){
        frame.can_id |= CAN_RTR_FLAG;
    }
    frame.can_dlc = 0;
    return send_message(frame);
}

int OdriveCan::receive_message(can_frame& frame) {
    return read(talk_socket, &frame, sizeof(frame));
}

int OdriveCan::odrv_can_id(cmd_id cmd){
    return ((axis_id<<5) | static_cast<int>(cmd));
}

int OdriveCan::e_stop(){
    send_message(cmd_id::ESTOP,false);
    return 0;
}

MotorError OdriveCan::get_motor_error(){
    send_message(cmd_id::MOTOR_ERROR,true);
    can_frame ans;
    receive_message(ans);
    uint64_t output_candidate;
    memcpy(&output_candidate,ans.data,ans.len);
    for (int i = 0; i<MOTOR_ERROR_LEN;i++){
        if (output_candidate == all_the_motor_errors[i]){
             return static_cast<MotorError>(output_candidate);
        }
    }
    char error_msg[50]; // Assuming 20 characters are sufficient for the hexadecimal representation
    sprintf(error_msg, "Received unexpected data frame  0x%lx", output_candidate); // Format output_candidate in hexadecimal
    throw UnexpectedMessageException(error_msg);
}

bool heartbeat_comparator(heartbeat_t expected, heartbeat_t actual){
    if (expected.axis_error != actual.axis_error){
        return false;
    }
    if (expected.axis_state != actual.axis_state){
        return false;
    }
    if (expected.controller_error_flag != actual.controller_error_flag){
        return false;
    }
    if (expected.encoder_error_flag != actual.encoder_error_flag){
        return false;
    }
    if (expected.motor_error_flag != actual.motor_error_flag){
        return false;
    }
    if (expected.trajectory_done != actual.trajectory_done){
        return false;
    }
    return true;
}
}
