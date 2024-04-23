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

#define ODRV_CAN_MASK 0x7e0

namespace odrive_can{


OdriveCan::OdriveCan(const std::string& interface, uint32_t axis_id_param)
{  
    can_socket = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (can_socket == -1) {
        throw std::runtime_error("Failed to create socket");
    }
    // Set the interface name
    struct ifreq ifr;
    std::strncpy(ifr.ifr_name, interface.c_str(), IFNAMSIZ);
    ioctl(can_socket, SIOCGIFINDEX, &ifr);

    // Bind the socket to the CAN interface
    struct sockaddr_can addr;
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;
    if (bind(can_socket, reinterpret_cast<struct sockaddr*>(&addr), sizeof(addr)) == -1) {
        close(can_socket);
        throw std::runtime_error("Failed to bind socket to interface");
    }

    // Set filter (accept only specific CAN IDs)
    can_filter filter[1];
    filter[0].can_id = axis_id_param<<5;
    filter[0].can_mask = ODRV_CAN_MASK;

    setsockopt(can_socket, SOL_CAN_RAW, CAN_RAW_FILTER, &filter, sizeof(filter));
    axis_id = axis_id_param;
    listening_thread = std::thread(&OdriveCan::listen_routine,this);
}
void OdriveCan::listen_routine(){
    return;
}

OdriveCan::~OdriveCan()
{   
    if (listening_thread.joinable()) {
            listening_thread.join();
        }
    close(can_socket);
}
int OdriveCan::send_message(const can_frame& frame) {
    return write(can_socket, &frame, sizeof(frame));
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
    return read(can_socket, &frame, sizeof(frame));
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
    return static_cast<MotorError>(ans.data[0]);
}



}

