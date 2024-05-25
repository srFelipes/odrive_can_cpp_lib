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
    thread_started =false;
    listening_thread = std::thread(&OdriveCan::listen_routine,this);
    while (!thread_started){}
    
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
    thread_started = true;
    while (listening){
        int ret = poll(pollfds, 1, 1); // Monitor indefinitely for events on the file descriptor
        if (ret > 0) {
            if (pollfds[0].revents & POLLIN) { // Check if the file descriptor is ready for reading
                // Read data from the socket
                ssize_t bytes_read = read(thread_soc, &last_frame, sizeof(can_frame));
                if (bytes_read < 0) {
                    // Handle error
                    std::cout<<"read error odrv_listen\n";
                } else {
                    uint16_t cmd = last_frame.can_id&ODRV_CMD_MASK;
                    cmd += 0;
                    if ((static_cast<uint16_t>(cmd_id::HEARTBEAT) == cmd)
                       & (last_frame.len == HEARTBEAT_LEN)){
                        last_heartbeat.axis_error =  last_frame.data[0]
                                                    +(last_frame.data[1]<<8)
                                                    +(last_frame.data[2]<<16)
                                                    +(last_frame.data[3]<<24);
                        memcpy(&last_heartbeat.axis_state,&last_frame.data[4],1);
                        last_heartbeat.controller_error_flag = last_frame.data[7]&1;
                        last_heartbeat.motor_error_flag = last_frame.data[5];
                        last_heartbeat.encoder_error_flag = last_frame.data[6];
                        last_heartbeat.trajectory_done = last_frame.data[7]>>7;
                    }
                    else if ((static_cast<uint16_t>(cmd_id::GET_ENCODER_ESTIMATES
                        ) == cmd)& last_frame.len == HEARTBEAT_LEN){
                            memcpy(&last_encoder_est.pos_estimate,last_frame.data,4);
                            memcpy(&last_encoder_est.vel_estimate,&last_frame.data[4],4);
                        }
                    else if (waiting_for_frame){
                        if (last_frame.can_id == expected_frame.can_id){
                            if  (last_frame.len == expected_frame.len){
                                memcpy(expected_frame.data,last_frame.data,8);
                                waiting_for_frame = false;
                            }
                            else{
                                waiting_for_frame_flags = -1;
                                waiting_for_frame = false;
                            }
                        }
                    }
                }
            }
        } else if (ret < 0) {

            std::cout<<"poll error odrv_listen\n";
            // Handle poll error
        }
    }
    close(thread_soc);
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
    frame.can_dlc = 0;
    if (is_rtr){
        frame.can_id |= CAN_RTR_FLAG;
    }
    return send_message(frame);
}

int OdriveCan::send_message(cmd_id command, can_frame frame_format){
    can_frame frame;
    frame.can_id = odrv_can_id(command);
    frame.can_dlc = 0;
    frame.can_id |= CAN_RTR_FLAG;
    expected_frame = frame_format;
    waiting_for_frame = true;
    waiting_for_frame_flags = 0;
    int result = send_message(frame);
    while (waiting_for_frame){}
    if (waiting_for_frame_flags == 0){
        return result;
    }
    else{
        return OdriveCan::send_message(command, frame_format);
    }
}

int OdriveCan::receive_message(can_frame& frame) {
    int result = read(talk_socket, &frame, sizeof(frame));

    if (result == 0){
        std::cout<<"read EOF error odrv_receive\n";
    }
    else if (result == -1){
        std::cout<<"read error error odrv_receive\n";
    }
    return result;
}

int OdriveCan::odrv_can_id(cmd_id cmd){
    return ((axis_id<<5) | static_cast<int>(cmd));
}

int OdriveCan::e_stop(){
    send_message(cmd_id::ESTOP,false);
    return 0;
}

MotorError OdriveCan::get_motor_error(){
    can_frame format;
    format.len = 8;
    format.can_id = odrv_can_id(cmd_id::MOTOR_ERROR);
    for (int i=0; i<N_OF_RETRIES; i++){
        send_message(cmd_id::MOTOR_ERROR,format);
        can_frame ans = expected_frame;
        uint64_t output_candidate;
        memcpy(&output_candidate,ans.data,ans.len);
        for (int i = 0; i<MOTOR_ERROR_LEN;i++){
            if (output_candidate == all_the_motor_errors[i]){
                return static_cast<MotorError>(output_candidate);
            }
        }
    }
    throw MaximumNumberOfRetriesReached("get_motor_error reached the maximum number of retries");
}

EncoderError OdriveCan::get_encoder_error(){
    can_frame format;
    format.len = 4;
    format.can_id = odrv_can_id(cmd_id::ENCODER_ERROR);
    send_message(cmd_id::ENCODER_ERROR,format);
    can_frame ans = expected_frame;
    EncoderError output_candidate;
    memcpy(&output_candidate,ans.data,ans.len);
    return output_candidate;
}
}
