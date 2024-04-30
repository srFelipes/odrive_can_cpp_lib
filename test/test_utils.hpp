#include <gtest/gtest.h>

#include <sys/socket.h>
#include <sys/ioctl.h>
#include <net/if.h>
#include <unistd.h>
#include <cstring>
#include <stdexcept>
#include <linux/can/raw.h>
#include <string>
#include <iostream>

#include <thread>
#include <chrono>
#include <atomic>
#include <poll.h>

#define TEST_DELAY 2

void test_sleep(){
    std::this_thread::sleep_for(std::chrono::milliseconds(TEST_DELAY));
}

void send_to_socket(int soc, can_frame frame){
        write(soc, &frame, sizeof(frame));
        test_sleep();
    }
bool can_frame_comparator(can_frame expected, can_frame actual){
    EXPECT_TRUE(expected.can_id == actual.can_id);
    EXPECT_TRUE(expected.len == actual.len);
    for (int i = 0; i< expected.len;i++){
        EXPECT_TRUE(expected.data[i] == actual.data[i]);
    }
    return true;
}
void heartbeat_comparator(odrive_can::heartbeat_t expected, odrive_can::heartbeat_t actual){
    EXPECT_TRUE(expected.axis_error == actual.axis_error) << "expected = " 
                << expected.axis_error << "|actual = " << actual.axis_error;

    EXPECT_TRUE (expected.axis_state == actual.axis_state)<< "expected = " 
                << static_cast<int>(expected.axis_state) << "|actual = " << static_cast<int>(actual.axis_state);

    EXPECT_TRUE (expected.controller_error_flag == actual.controller_error_flag)<< "expected = " 
                << expected.controller_error_flag << "|actual = " << actual.controller_error_flag;

    EXPECT_TRUE (expected.encoder_error_flag == actual.encoder_error_flag)<< "expected = " 
                << expected.encoder_error_flag << "|actual = " << actual.encoder_error_flag;

    EXPECT_TRUE (expected.motor_error_flag == actual.motor_error_flag)<< "expected = " 
                << expected.motor_error_flag << "|actual = " << actual.motor_error_flag;

    EXPECT_TRUE (expected.trajectory_done == actual.trajectory_done)<< "expected = " 
                << expected.trajectory_done << "|actual = " << actual.trajectory_done;
}

int create_socket(){

    struct sockaddr_can addr;
    struct ifreq ifr;
    std::string interface = "vcan0" ;
    int output_socket;
    output_socket = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (output_socket == -1) {
        throw std::runtime_error("Failed to create socket");
    }
    // Set the interface name
    
    std::strncpy(ifr.ifr_name, interface.c_str(), IFNAMSIZ);
    ioctl(output_socket, SIOCGIFINDEX, &ifr);

    // Bind the socket to the CAN interface
    
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;
    if (bind(output_socket, reinterpret_cast<struct sockaddr*>(&addr), sizeof(addr)) == -1) {
        close(output_socket);
        throw std::runtime_error("Failed to bind socket to interface");
    }
    return output_socket;
}

void wait_for_msg_and_answer(can_frame msg_to_wait, can_frame* answer, int answer_length){
        int soc = create_socket();
        can_frame last_msg;
        int read_size;
        bool break_the_loop = true;
        
        bool last_read = true;
        struct pollfd pollfds[1];
        pollfds[0].fd = soc; // Set the file descriptor to monitor
        pollfds[0].events = POLLIN; // Set the events to monitor for (in this case, readability)

        while (true){
            int ret = poll(pollfds, 1, TEST_DELAY); // Monitor indefinitely for events on the file descriptor
            if (ret > 0) {
                if (pollfds[0].revents & POLLIN) { // Check if the file descriptor is ready for reading
                    // Read data from the socket
                    ssize_t bytes_read = read(soc, &last_msg, sizeof(can_frame));

                    if (bytes_read < 0) {
                        // Handle error
                        std::cout<<"read error wait_for_msg\n";
                    } else {
                        // Increment message count or handle received data
                        if ((msg_to_wait.can_id == last_msg.can_id)
                            &msg_to_wait.len == last_msg.len){
                                bool should_send = true;
                            for (int i = 0; i< msg_to_wait.len;i++){
                                should_send &= (msg_to_wait.data[i] == last_msg.data[i]);
                            }
                            if (should_send){
                                for (int k = 0; k< answer_length; k++){
                                    send_to_socket(soc,answer[k]);
                                }
                                break;
                            }
                        }
                    }
                }
            } else if (ret < 0) {
                // Handle poll error
                std::cout<<"poll error wait_for_msg\n";
            }
        }
        close(soc);
    }