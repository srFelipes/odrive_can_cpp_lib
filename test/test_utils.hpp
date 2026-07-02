#pragma once
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
#include <vector>

#include <thread>
#include <chrono>
#include <atomic>
#include <poll.h>

#define TEST_DELAY 15

bool operator==(const can_frame &lhs, const can_frame &rhs) {
    return (lhs.can_id == rhs.can_id &&
         lhs.can_dlc == rhs.can_dlc &&
         std::memcmp(lhs.data, rhs.data, lhs.can_dlc) == 0);
}

bool operator!=(const can_frame &lhs, const can_frame &rhs) {
    return !(lhs == rhs);
}

void test_sleep(){
    std::this_thread::sleep_for(std::chrono::milliseconds(TEST_DELAY));
}

void send_to_socket(int soc, can_frame frame){
    int size = sizeof(frame);
    int result = 0;
    while (result!=size){
        result =  write(soc, &frame, sizeof(frame));
    }
    }
bool can_frame_comparator(can_frame expected, can_frame actual){
    EXPECT_TRUE(expected.can_id == actual.can_id)<< "expected = " 
                << expected.can_id << "|actual = " << actual.can_id;
    EXPECT_TRUE(expected.len == actual.len)<< "expected = " 
                << static_cast<int>(expected.len) << "|actual = " << static_cast<int>(actual.len);
    for (int i = 0; i< expected.len;i++){
        EXPECT_TRUE(expected.data[i] == actual.data[i]) << "i = " << i << " | expected = " 
                << static_cast<int>(expected.data[i])  << " | actual = " << static_cast<int>(actual.data[i]) ;
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

void answer(can_frame petition,
            std::vector<can_frame> answers, 
            int retries,
            bool &initialization_done,
            bool &end_answer){
    if (!initialization_done ||
        (retries < 0)){
        return;
    }
    int soc = create_socket();
    can_frame last_msg;
    int read_size;
    bool break_the_loop = true;
    
    bool last_read = true;
    struct pollfd pollfds[1];
    pollfds[0].fd = soc; // Set the file descriptor to monitor
    pollfds[0].events = POLLIN; // Set the events to monitor for (in this case, readability)
    int k = 0;
    initialization_done = true;
    while (end_answer){
        int ret = poll(pollfds, 1, TEST_DELAY); // Monitor indefinitely for events on the file descriptor
        if (ret > 0) {
            if (pollfds[0].revents & POLLIN) { // Check if the file descriptor is ready for reading
                // Read data from the socket
                ssize_t bytes_read = read(soc, &last_msg, sizeof(can_frame));

                if (bytes_read < 0) {
                    // Handle error
                    std::cout<<"read error wait_for_msg\n";
                    return;
                } else {
                    if (petition == last_msg){
                        if (retries == 1){
                            for (can_frame cf: answers){
                                send_to_socket(soc,cf);
                            }
                        }
                        else{
                            retries--;
                        }
                    }
                }
            }
        }
    }
}
void wait_for_msg_and_answer(
    can_frame msg_to_wait,
    can_frame* answer, 
    int answer_length, 
    bool answer_per_petition, 
    std::atomic<bool>* flag){
    int soc = create_socket();
    can_frame last_msg;
    int read_size;
    bool break_the_loop = true;
    
    bool last_read = true;
    struct pollfd pollfds[1];
    pollfds[0].fd = soc; // Set the file descriptor to monitor
    pollfds[0].events = POLLIN; // Set the events to monitor for (in this case, readability)
    int k = 0;
    while (*flag){
        int ret = poll(pollfds, 1, TEST_DELAY); // Monitor indefinitely for events on the file descriptor
        if (ret > 0) {
            if (pollfds[0].revents & POLLIN) { // Check if the file descriptor is ready for reading
                // Read data from the socket
                ssize_t bytes_read = read(soc, &last_msg, sizeof(can_frame));

                if (bytes_read < 0) {
                    // Handle error
                    std::cout<<"read error wait_for_msg\n";
                } else {
                    if ((msg_to_wait.can_id == last_msg.can_id)
                        &msg_to_wait.len == last_msg.len){
                            bool should_send = true;
                        for (int i = 0; i< msg_to_wait.len;i++){
                            should_send &= (msg_to_wait.data[i] == last_msg.data[i]);
                        }
                        if (should_send){
                            if (answer_per_petition){
                                send_to_socket(soc,answer[k]);
                                k++;
                                if (k==answer_length){
                                    break;
                                }
                            }
                            else{
                            while (k < answer_length){
                                send_to_socket(soc,answer[k]);
                                k++;
                            }
                            break;
                            }
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

void listener(
    std::vector<can_frame>* buffer,
    std::atomic <bool>* listening,
    std::atomic <bool>* ready,
    std::atomic <int> *num_of_calls){
    int soc = create_socket();
    std::atomic <int> default_nofmsgs;
    std::atomic <int> &n_of_msgs = (num_of_calls) ? *num_of_calls: default_nofmsgs;
    n_of_msgs = 0;
    int poll_size;
    int read_bytes;
    can_frame last_message;
    int frame_size = sizeof(can_frame);

    struct pollfd pollfds[1];
    pollfds[0].fd = soc; // Set the file descriptor to monitor
    pollfds[0].events = POLLIN; // Set the events to monitor for (in this case, readability)

    *ready = true;
    while (*listening){
        poll_size =  poll(pollfds,1,TEST_DELAY);
        if ((poll_size>0) & (pollfds[0].revents & POLLIN)){
            read_bytes = read(soc, &last_message, frame_size);
            if (read_bytes>0){
                buffer->push_back(std::move(last_message));
                n_of_msgs++;
            }
            else if (read_bytes<0){
                //handle read error
            }
        }
        else if (poll_size < 0) {
            // Handle poll error
            std::cout<<"poll error wait_for_msg\n";
        }
    }
    close(soc);
}

/**
 * @brief this functions blocks until either condition returns true
 * or timeout milliseconds have passed since it was called
 * 
 * @param condition a boolean expresion to be evaluated, is it returns true it braks
 * @param timeout in ms, breaks when this amount of ms have passed since this function
 *  is called
 * @return int 
 */
int wait_for_condition_with_timeout(const std::function <bool()> & condition,
                                    int timeout){
    auto start_time = std::chrono::steady_clock::now();
    auto chrono_timeout = std::chrono::milliseconds(timeout);
    auto current_time = start_time;
    while (true){
        auto diff = std::chrono::steady_clock::now()-start_time;
        if (std::chrono::duration_cast<std::chrono::milliseconds>(diff)
            > chrono_timeout){
            break;
        }
        if (condition()){
            return 0;
        }
    }
    return -1;
}


/**
 * @brief Test utility that listens on vcan0 and responds to incoming CAN frames.
 *        Runs in its own thread, waits for a specific trigger frame, and replies
 *        with one or all of the provided response frames depending on the mode.
 * 
 * @param messages Vector where messages[0] is the trigger frame to wait for,
 *                 and messages[1..n] are the response frames to send upon a match.
 * @param thread_started Set to true once the socket is bound and the poll loop
 *                       is ready to receive — caller should spin-wait on this
 *                       before proceeding.
 * @param thread_listening Controls the poll loop lifetime. Set to false from the
 *                         calling thread to stop the listener and exit cleanly.
 * @param message_per_request If true, sends one response frame per trigger match
 *                            (messages[1], then messages[2], etc.) and exits when
 *                            all responses are exhausted. If false, sends all
 *                            response frames on every trigger match.
 */
void listen_and_answer(
    const std::vector<can_frame> &messages,
    std::atomic <bool> &thread_started,
    std::atomic <bool> &thread_listening,
    bool message_per_request
){
    thread_started = false;
    int soc = create_socket();
    
    can_frame expected_msg = messages[0];
    int poll_size;
    int read_bytes;
    can_frame last_message;
    int frame_size = sizeof(can_frame);

    struct pollfd pollfds[1];
    pollfds[0].fd = soc; // Set the file descriptor to monitor
    pollfds[0].events = POLLIN; // Set the events to monitor for (in this case, readability)
    int current_message_index = 1;
    thread_started = true;
    while (thread_listening){
        poll_size =  poll(pollfds,1,TEST_DELAY);
        if ((poll_size>0) & (pollfds[0].revents & POLLIN)){
            read_bytes = read(soc, &last_message, frame_size);
            if (read_bytes>0){
                if (last_message == expected_msg){
                    if (message_per_request){
                        write(soc,
                              &messages[current_message_index],
                              frame_size);
                        current_message_index++;
                        if (current_message_index == messages.size()){break;}
                    }
                    else{
                        for (int i=1; i<messages.size(); i++){
                            write(soc,
                                &messages[i],
                                frame_size);
                        }
                    }
                }
            }
            else if (read_bytes<0){
                //handle read error
            }
        }
        else if (poll_size < 0) {
            // Handle poll error
            std::cout<<"poll error wait_for_msg\n";
        }
    }
    close(soc);

}