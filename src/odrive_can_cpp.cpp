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
namespace odrive_can{

OdriveCan::OdriveCan(const std::string& interface, uint32_t filterId)
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
    filter[0].can_id = filterId;
    filter[0].can_mask = CAN_SFF_MASK;

    setsockopt(can_socket, SOL_CAN_RAW, CAN_RAW_FILTER, &filter, sizeof(filter));

}

OdriveCan::~OdriveCan()
{
    close(can_socket);
}
int OdriveCan::send_message(const can_frame& frame) {
    return write(can_socket, &frame, sizeof(frame));
}

int OdriveCan::receive_message(can_frame& frame) {
    return read(can_socket, &frame, sizeof(frame));
}

}

