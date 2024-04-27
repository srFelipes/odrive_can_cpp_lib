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
#include <chrono>
#include <atomic>
#include <poll.h>
#define BUFFER_SIZE 30
#define TEST_DELAY 2

void test_sleep(){
    std::this_thread::sleep_for(std::chrono::milliseconds(TEST_DELAY));
}

TEST(importingTests,importLib){
    #ifdef ODRIVE_CAN
    EXPECT_TRUE(true);

    #else 
    EXPECT_TRUE(false);
    #endif
}

TEST(importingTest,canUp){
    EXPECT_NO_THROW(odrive_can::OdriveCan odrv("vcan0",0x69);odrv.listening = false;);
    
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
        end_listening_flag = false;
    }
    void TearDown(){
        odrv.listening = false;
        end_listening_flag = true;
        close(can_socket);
        if (listening_thread.joinable()) {
            listening_thread.join();
        }
    }

    void listen_routine(void){
        bool last_read = true;
        struct pollfd pollfds[1];
        pollfds[0].fd = can_socket; // Set the file descriptor to monitor
        pollfds[0].events = POLLIN; // Set the events to monitor for (in this case, readability)

        while (last_read){
            last_read = !end_listening_flag;
            int ret = poll(pollfds, 1, TEST_DELAY); // Monitor indefinitely for events on the file descriptor
            if (ret > 0) {
                if (pollfds[0].revents & POLLIN) { // Check if the file descriptor is ready for reading
                    // Read data from the socket
                    ssize_t bytes_read = read(can_socket, &msg_buffer[msg_count], sizeof(can_frame));
                    if (bytes_read < 0) {
                        // Handle error
                        std::cout<<"read error listen_routine\n";
                    } else {
                        // Increment message count or handle received data
                        msg_count++;
                    }
                }
            } else if (ret < 0) {
                // Handle poll error
                std::cout<<"poll error listen_routine\n";
            }
        }
    }
    
    void send_to_socket(int soc, can_frame frame){
        write(soc, &frame, sizeof(frame));
        test_sleep();
    }

    commandsTest(): odrv(interface,axis_id){}

    struct sockaddr_can addr;
    int can_socket;
    struct ifreq ifr;
    std::string interface = "vcan0" ;
    uint32_t axis_id = 4; //no particular reason for this number
    
    can_frame msg_buffer[BUFFER_SIZE];
    int msg_count = 0;
    bool end_listening_flag = false;
    std::thread listening_thread;
    odrive_can::OdriveCan odrv;
    public:
    void wait_for_msg_and_answer(can_frame msg_to_wait, can_frame answer){
        int soc = create_socket();
        can_frame last_msg;
        int read_size;
        bool break_the_loop = true;
        
        bool last_read = true;
        struct pollfd pollfds[1];
        pollfds[0].fd = soc; // Set the file descriptor to monitor
        pollfds[0].events = POLLIN; // Set the events to monitor for (in this case, readability)

        while (last_read){
            last_read = !end_listening_flag;
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
                        if (can_frame_comparator(msg_to_wait,last_msg)){
                            send_to_socket(soc,answer);
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
};

TEST_F(commandsTest, e_stop){
    int result = odrv.e_stop();
    EXPECT_TRUE(0 == result);
    end_listening_flag = true;
    odrv.listening = false;
    listening_thread.join();
    EXPECT_TRUE(1 == msg_count);

    can_frame expected;
    expected.can_id = (axis_id<<5)|0x002; //cmd_id = 2 ESTOP
    expected.len = 0;

    EXPECT_TRUE(expected.can_id == msg_buffer[0].can_id);
    EXPECT_TRUE(expected.len == msg_buffer[0].len);
}

TEST_F(commandsTest,get_motor_error_no_error){
    
    can_frame expected_petition;
    expected_petition.can_id = (4<<5)|0x003; //4 is odrv id, 3 is cmd_id
    expected_petition.can_id |= CAN_RTR_FLAG;
    expected_petition.len = 0;

    can_frame given_answer;
    given_answer.can_id = (4<<5)|0x003; //4 is odrv id, 3 is cmd_id
    given_answer.len = 8;
    given_answer.data[0] = 0;    //MotorError.NONE;
    memset(&given_answer.data,0,8);

    std::thread wfmaa_thread(&commandsTest::wait_for_msg_and_answer,this,
                expected_petition,given_answer);
    test_sleep(); //this delay is so the thread can start TODO, use a lock until the socket is created
    MotorError output = odrv.get_motor_error();
    end_listening_flag = true;
    if (wfmaa_thread.joinable()){
        wfmaa_thread.join();
    }
    EXPECT_TRUE(can_frame_comparator(expected_petition, msg_buffer[0]));
    EXPECT_TRUE(can_frame_comparator(given_answer, msg_buffer[1]));
    EXPECT_TRUE(MotorError::NONE == output);
}

TEST_F(commandsTest,get_motor_error_phase_r_out_of_r){
    
    can_frame expected_petition;
    expected_petition.can_id = (4<<5)|0x003; //4 is odrv id, 3 is cmd_id
    expected_petition.can_id |= CAN_RTR_FLAG;
    expected_petition.len = 0;

    can_frame given_answer;
    given_answer.can_id = (4<<5)|0x003; //4 is odrv id, 3 is cmd_id
    given_answer.len = 8;
    memset(&given_answer.data,0,8);
    given_answer.data[0] = 1;    //MotorError.NONE;

    std::thread wfmaa_thread(&commandsTest::wait_for_msg_and_answer,this,
                expected_petition,given_answer);
    test_sleep(); //this delay is so the thread can start TODO, use a lock until the socket is created
    MotorError output = odrv.get_motor_error();
    end_listening_flag = true;
    odrv.listening = false;
    
    if (wfmaa_thread.joinable()){
        wfmaa_thread.join();
    }
    EXPECT_TRUE(can_frame_comparator(expected_petition, msg_buffer[0]));
    EXPECT_TRUE(can_frame_comparator(given_answer, msg_buffer[1]));
    EXPECT_TRUE(MotorError::PHASE_RESISTANCE_OUT_OF_RANGE == output);
}

TEST_F(commandsTest,get_motor_error_answer_unknown){
    can_frame expected_petition;
    expected_petition.can_id = (4<<5)|0x003; //4 is odrv id, 3 is cmd_id
    expected_petition.can_id |= CAN_RTR_FLAG;
    expected_petition.len = 0;

    can_frame given_answer;
    given_answer.can_id = (4<<5)|0x003; //4 is odrv id, 3 is cmd_id
    given_answer.len = 8;
    memset(&given_answer.data,0,8);
    given_answer.data[0] = 3;    //MotorError. non existent;

    std::thread wfmaa_thread(&commandsTest::wait_for_msg_and_answer,this,
                expected_petition,given_answer);
    test_sleep(); //this delay is so the thread can start TODO, use a lock until the socket is created
    MotorError output;
    EXPECT_THROW(output = odrv.get_motor_error(),UnexpectedMessageException);
    end_listening_flag = true;
    odrv.listening = false;
    
    if (wfmaa_thread.joinable()){ 
        wfmaa_thread.join();
    }
    EXPECT_TRUE(can_frame_comparator(expected_petition, msg_buffer[0]));
    EXPECT_TRUE(can_frame_comparator(given_answer, msg_buffer[1]));
}

TEST_F(commandsTest,get_motor_error_max_value){
    can_frame expected_petition;
    expected_petition.can_id = (4<<5)|0x003; //4 is odrv id, 3 is cmd_id
    expected_petition.can_id |= CAN_RTR_FLAG;
    expected_petition.len = 0;

    can_frame given_answer;
    given_answer.can_id = (4<<5)|0x003; //4 is odrv id, 3 is cmd_id
    given_answer.len = 8;
    memset(&given_answer.data,0,8);
    given_answer.data[4] = 8;    //MotorError.UNBALANCED_PHASES;

    std::thread wfmaa_thread(&commandsTest::wait_for_msg_and_answer,this,
                expected_petition,given_answer);
    test_sleep(); //this delay is so the thread can start TODO, use a lock until the socket is created
    MotorError output;
    output = odrv.get_motor_error();
    end_listening_flag = true;
    odrv.listening = false;
    
    if (wfmaa_thread.joinable()){
        wfmaa_thread.join();
    }
    EXPECT_TRUE(can_frame_comparator(expected_petition, msg_buffer[0]));
    EXPECT_TRUE(can_frame_comparator(given_answer, msg_buffer[1]));
    EXPECT_TRUE(MotorError::UNBALANCED_PHASES == output);
}

TEST_F(commandsTest,periodic_messages_heartbeat_3_states){
    can_frame heartbeat_msg;
    heartbeat_msg.can_id = (4<<5)|0x001; //4 is odrv id, 1 is cmd_id
    heartbeat_msg.len = 8;
    memset(&heartbeat_msg.data,0,8);
    heartbeat_msg.data[4] = 1;
    int talk_soc = create_socket();
    //FIRST MSG
    send_to_socket(talk_soc,heartbeat_msg);
    test_sleep();
    odrive_can::heartbeat_t expected;
    expected.axis_error = static_cast<uint32_t>(AxisError::NONE);
    expected.axis_state = AxisState::IDLE;
    expected.controller_error_flag = false;
    expected.encoder_error_flag = false;
    expected.motor_error_flag = false;
    expected.trajectory_done = false;
    
    heartbeat_comparator(expected,odrv.last_heartbeat);

    //SECOND MSG
    heartbeat_msg.data[4] = 2;
    send_to_socket(talk_soc,heartbeat_msg);
    test_sleep();
    expected.axis_state = AxisState::STARTUP_SEQUENCE;
    heartbeat_comparator(expected,odrv.last_heartbeat);

    //THIRD MSG
    heartbeat_msg.data[4] = 3;
    send_to_socket(talk_soc,heartbeat_msg);
    test_sleep();
    expected.axis_state = AxisState::FULL_CALIBRATION_SEQUENCE;
    heartbeat_comparator(expected,odrv.last_heartbeat);    
}

TEST_F(commandsTest,periodic_messages_heartbeat_flags){
    can_frame heartbeat_msg;
    heartbeat_msg.can_id = (4<<5)|0x001; //4 is odrv id, 1 is cmd_id
    heartbeat_msg.len = 8;
    memset(&heartbeat_msg.data,0,8);
    heartbeat_msg.data[4] = 1;
    heartbeat_msg.data[5] = 1;
    heartbeat_msg.data[6] = 1;
    heartbeat_msg.data[7] = 0x81;
    int talk_soc = create_socket();
    //FIRST MSG
    send_to_socket(talk_soc,heartbeat_msg);
    test_sleep();
    odrive_can::heartbeat_t expected;
    expected.axis_error = static_cast<uint32_t>(AxisError::NONE);
    expected.axis_state = AxisState::IDLE;
    expected.controller_error_flag = true;
    expected.encoder_error_flag = true;
    expected.motor_error_flag = true;
    expected.trajectory_done = true;
    heartbeat_comparator(expected,odrv.last_heartbeat);
}

TEST_F(commandsTest,periodic_messages_heartbeat_all_axis_errors){
    can_frame heartbeat_msg;
    heartbeat_msg.can_id = (4<<5)|0x001; //4 is odrv id, 1 is cmd_id
    heartbeat_msg.len = 8;
    memset(&heartbeat_msg.data,0,8);
    heartbeat_msg.data[0] = 0xc1;
    heartbeat_msg.data[1] = 0x7b;
    heartbeat_msg.data[2] = 0xe;
    heartbeat_msg.data[4] = 1;
    int talk_soc = create_socket();
    //FIRST MSG
    send_to_socket(talk_soc,heartbeat_msg);
    test_sleep();
    odrive_can::heartbeat_t expected;
    expected.axis_error = 0xe7bc1;
    expected.axis_state = AxisState::IDLE;
    expected.controller_error_flag = false;
    expected.encoder_error_flag = false;
    expected.motor_error_flag = false;
    expected.trajectory_done = false;
    heartbeat_comparator(expected,odrv.last_heartbeat);
}

TEST_F(commandsTest,periodic_heartbeat_strange_msg_is_ignored){
    can_frame heartbeat_msg;
    heartbeat_msg.can_id = (4<<5)|0x001; //4 is odrv id, 1 is cmd_id
    heartbeat_msg.len = 7;
    memset(&heartbeat_msg.data,0,8);
    int talk_soc = create_socket();
    odrive_can::heartbeat_t dummy_heartbeat;
    dummy_heartbeat.axis_error = 555;
    dummy_heartbeat.axis_state = AxisState::ENCODER_HALL_POLARITY_CALIBRATION;
    dummy_heartbeat.controller_error_flag = false;
    dummy_heartbeat.encoder_error_flag = true;
    dummy_heartbeat.motor_error_flag = false;
    dummy_heartbeat.trajectory_done = true;
    odrv.last_heartbeat = dummy_heartbeat;
    send_to_socket(talk_soc,heartbeat_msg);
    test_sleep();
    heartbeat_comparator(dummy_heartbeat,odrv.last_heartbeat);
    EXPECT_TRUE(can_frame_comparator(heartbeat_msg,odrv.last_frame));
}


int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv); 
    return RUN_ALL_TESTS();
}