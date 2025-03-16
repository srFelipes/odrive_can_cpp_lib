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
#include <linux/can/raw.h>
#include <chrono>

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

    // Set filter (accept only specific CAN IDs)
    can_filter filter[1] = {cf_filter};
    setsockopt(i_listening_socket, SOL_CAN_RAW, CAN_RAW_FILTER, &filter, sizeof(filter));

    while (b_listening){
        int i_n_of_events = poll(pollfds, 1, 1); // Monitor indefinitely for events on the file descriptor
        if (i_n_of_events > 0){
            if (pollfds[0].revents & POLLIN) {
                ssize_t bytes_read = read(i_listening_socket, &last_frame, sizeof(can_frame));
                if (bytes_read < 0) {
                    // Handle error
                    std::cout<<"read error odrv_listen\n";
                }
                else{
                    if (ask_state == SENDING_REQUEST){ 
                        if ((cf_waiting_for.can_id | CAN_RTR_FLAG) == last_frame.can_id){
                            ask_state = WAITING_RESPONSE;
                        }
                    }
                    else if (ask_state == WAITING_RESPONSE){
                        if (cf_waiting_for.can_id == last_frame.can_id){
                            cf_waiting_for = last_frame;
                            ask_state = NOT_ASKING;
                        }
                    }
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
        i_talking_socket = create_socket(s_interface_name);
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
    if (write(i_talking_socket, &frame,sizeof(can_frame)) != -1){
        return 0;
    };
    return -1;
}

bool Messenger::ask(can_frame &command, int timeout){
    can_frame petition = command;
    petition.can_id |= CAN_RTR_FLAG;
    cf_waiting_for = command;
    auto chrono_timeout = std::chrono::milliseconds(timeout);
    ask_state = SENDING_REQUEST;
    send(petition);
    auto start_time = std::chrono::steady_clock::now();
    auto current_time = start_time;
    bool return_val = true;
    while(ask_state){
        auto diff = std::chrono::steady_clock::now()-start_time;
        if (std::chrono::duration_cast<std::chrono::milliseconds>
            (diff) > chrono_timeout){
                return_val = false;
                break;
        }
    };
    command = cf_waiting_for;
    return return_val;
}

bool Messenger::is_listening(){
    return b_listening;
}
