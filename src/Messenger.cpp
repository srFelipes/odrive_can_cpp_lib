#include "Messenger.hpp"
#include <time.h>
#include <iostream>

#include <sys/socket.h>
#include <sys/ioctl.h>
#include <net/if.h>
#include <unistd.h>
#include <stdio.h>
#include <string>
#include <cstring>
#include <poll.h>


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

void Messenger::listening_routine(){
    i_listening_socket = create_socket(s_interface_name);
    struct pollfd pollfds[1];
    pollfds[0].fd = i_listening_socket; // Set the file descriptor to monitor
    pollfds[0].events = POLLIN; // Set the events to monitor for (in this case, readability)
    b_thread_started = true;
    while (b_listening){
        int i_n_of_events = poll(pollfds, 1, 1); // Monitor indefinitely for events on the file descriptor
        if (i_n_of_events > 0){
            if (pollfds[0].revents & POLLIN) {
                ssize_t bytes_read = read(i_listening_socket, &last_frame, sizeof(can_frame));
                if (bytes_read < 0) {
                    // Handle error
                    std::cout<<"read error odrv_listen\n";
                } else{
                    callback();
                }
            }
        }
        else if(i_n_of_events < 0){
            std::cout<<"poll error odrv_listen\n";
            // Handle poll error
        }
    }
}

Messenger::Messenger(const std::string& interface, can_filter filter):
    s_interface_name(interface),
    cf_filter(filter){
        b_listening = false;
    }
Messenger::~Messenger(){
    thread_kill();
}
bool Messenger::thread_start(){
    b_listening = true;
    b_thread_started = false;
    th_listening_thread = std::thread(&Messenger::listening_routine, this);
    while(!b_thread_started){};
    return true;
}
void Messenger::thread_kill(){
    if (th_listening_thread.joinable()){
        b_listening = false;
        th_listening_thread.join();
    }
}
int Messenger::send(can_frame frame){
    return -1;
}
int Messenger::ask(can_frame command){
    return -1;
}
bool Messenger::is_listening(){
    return b_listening;
}
int Messenger::stop(){
    b_listening = false;
    return -1;
}
int Messenger::restart(){
    b_listening = true;
    return -1;
}