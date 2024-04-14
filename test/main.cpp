#include <gtest/gtest.h>
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

#include <thread>

#define BUFFER_SIZE 30

TEST(importingTests,importLib){
    #ifdef ODRIVE_CAN
    EXPECT_TRUE(true);

    #else 
    EXPECT_TRUE(false);
    #endif
}

TEST(importingTest,canUp){
    EXPECT_NO_THROW(odrive_can::OdriveCan odrv("vcan0",0x69));
}

bool can_frame_comparator(can_frame expected, can_frame actual){
    if (expected.can_id != actual.can_id){
        return false;
    }
    if (expected.len != actual.len){
        return false;
    }
    for (int i = 0; i< expected.len;i++){
        if (expected.data[i] != actual.data[i]){
            return false;
        }
    }
    return true;
}

class commandsTest : public testing::Test{
    protected:
    int create_socket(){
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
    void SetUp(){
        can_socket = create_socket();
        listening_thread = std::thread(&commandsTest::listen_routine, this);
        // odrv = odrive_can::OdriveCan(interface,axis_id);
        
    }
    void TearDown(){
        close(can_socket);
    }

    void listen_routine(void){
        while (!end_listening_flag){
            read(can_socket, &msg_buffer[msg_count], sizeof(can_frame));
            msg_count++;
        }        
    }
    
    void send_to_socket(int soc, can_frame frame){
        write(soc, &frame, sizeof(frame));
    }

    void wait_for_msg_and_answer(can_frame msg_to_wait, can_frame answer){
        int soc = create_socket();
        can_frame last_msg;
        while (!end_listening_flag){
            read(can_socket, &last_msg, sizeof(can_frame));
            if (can_frame_comparator(msg_to_wait,last_msg)){
                send_to_socket(soc,answer);
            }
        }
    }

    struct sockaddr_can addr;
    int can_socket;
    struct ifreq ifr;
    std::string interface = "vcan0" ;
    uint32_t axis_id = 4; //no particular reason for this number
    
    can_frame msg_buffer[BUFFER_SIZE];
    int msg_count;
    bool end_listening_flag = false;
    std::thread listening_thread;
};

TEST_F(commandsTest, e_stop){
    odrive_can::OdriveCan odrv(interface,axis_id);
    int result = odrv.e_stop();
    EXPECT_TRUE(0 == result);
    end_listening_flag = true;
    listening_thread.join();
    EXPECT_TRUE(1 == msg_count);

    can_frame expected;
    expected.can_id = (axis_id<<5)|0x002; //cmd_id = 2 ESTOP
    expected.len = 0;

    EXPECT_TRUE(expected.can_id == msg_buffer[0].can_id);
    EXPECT_TRUE(expected.len == msg_buffer[0].len);
}

TEST_F(commandsTest,get_motor_error_no_error){
    odrive_can::OdriveCan odrv(interface,axis_id);
    int talker = create_socket();
    MotorError output = odrv.get_motor_error();

}

int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv); 
    return RUN_ALL_TESTS();
}