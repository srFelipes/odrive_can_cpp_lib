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
    EXPECT_NO_THROW(odrive_can::OdriveCan odrv = odrive_can::OdriveCan("vcan0",0x69));
}

class commandsTest : public testing::Test{
    protected:

    void SetUp(){
        can_socket = socket(PF_CAN, SOCK_RAW, CAN_RAW);
        if (can_socket == -1) {
            throw std::runtime_error("Failed to create socket");
        }
        // Set the interface name
        
        std::strncpy(ifr.ifr_name, interface.c_str(), IFNAMSIZ);
        ioctl(can_socket, SIOCGIFINDEX, &ifr);

        // Bind the socket to the CAN interface
        
        addr.can_family = AF_CAN;
        addr.can_ifindex = ifr.ifr_ifindex;
        if (bind(can_socket, reinterpret_cast<struct sockaddr*>(&addr), sizeof(addr)) == -1) {
            close(can_socket);
            throw std::runtime_error("Failed to bind socket to interface");
        }
        listening_thread = std::thread(&commandsTest::listen_routine, this);
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


    struct sockaddr_can addr;
    int can_socket;
    struct ifreq ifr;
    std::string interface = "vcan0" ;
    uint32_t axis_id = 4; //no particular reason for this number
    odrive_can::OdriveCan odrv = odrive_can::OdriveCan(interface,axis_id);
    can_frame msg_buffer[BUFFER_SIZE];
    int msg_count;
    bool end_listening_flag = false;
    std::thread listening_thread;
};

TEST_F(commandsTest,e_stop){
    int result = odrv.e_stop();
    EXPECT_TRUE(0 == result);
    end_listening_flag = true;
    listening_thread.join();
    EXPECT_TRUE(1 == msg_count);

    can_frame expected;
    expected.can_id = (axis_id<<5)|0x002; //cmd_id = 2 ESTOP
    expected.can_dlc = 0;

    EXPECT_TRUE(expected.can_id == msg_buffer[0].can_id);
    EXPECT_TRUE(expected.can_dlc == msg_buffer[0].can_dlc);




}

int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv); 
    return RUN_ALL_TESTS();
}