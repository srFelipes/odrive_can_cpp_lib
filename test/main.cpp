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

#include "test_utils.hpp"
#define BUFFER_SIZE 300

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

class commandsTest : public testing::Test{
    protected:
    
    void SetUp(){
        can_socket = create_socket();
        fixture_listening = true;
        listening_thread = std::thread(&commandsTest::listen_routine, this);
        test_sleep();
    }
    void TearDown(){
        odrv.listening = false;
        fixture_listening = false;
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

        while (fixture_listening){
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
    commandsTest(): odrv("vcan0",axis_id){}
    int can_socket;    
    uint32_t axis_id = 4; //no particular reason for this number
    
    can_frame msg_buffer[BUFFER_SIZE];
    int msg_count = 0;
    bool fixture_listening = true;
    std::thread listening_thread;
    odrive_can::OdriveCan odrv;
    public:
};

TEST_F(commandsTest, e_stop){
    int result = odrv.e_stop();
    EXPECT_TRUE(0 == result);
    fixture_listening = false;
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

    std::thread wfmaa_thread(wait_for_msg_and_answer,
                expected_petition,&given_answer,1,false,&fixture_listening);
    test_sleep(); //this delay is so the thread can start TODO, use a lock until the socket is created
    MotorError output = odrv.get_motor_error();
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

    std::thread wfmaa_thread(wait_for_msg_and_answer,
                expected_petition,&given_answer,1,false,&fixture_listening);
    test_sleep(); //this delay is so the thread can start TODO, use a lock until the socket is created
    MotorError output = odrv.get_motor_error();
    
    if (wfmaa_thread.joinable()){
        wfmaa_thread.join();
    }
    EXPECT_TRUE(can_frame_comparator(expected_petition, msg_buffer[0]));
    EXPECT_TRUE(can_frame_comparator(given_answer, msg_buffer[1]));
    EXPECT_TRUE(MotorError::PHASE_RESISTANCE_OUT_OF_RANGE == output);
}

TEST_F(commandsTest,get_motor_error_answer_unknown_no_throw){
    can_frame expected_petition;
    expected_petition.can_id = (4<<5)|0x003; //4 is odrv id, 3 is cmd_id
    expected_petition.can_id |= CAN_RTR_FLAG;
    expected_petition.len = 0;

    can_frame unknown_answer;
    unknown_answer.can_id = (4<<5)|0x003; //4 is odrv id, 3 is cmd_id
    unknown_answer.len = 8;
    memset(&unknown_answer.data,0,8);
    unknown_answer.data[0] = 3;    //MotorError. non existent;

    can_frame good_answer; // data is 0, so error is NONE
    good_answer.can_id = (4<<5)|0x003; //4 is odrv id, 3 is cmd_id
    good_answer.len = 8;
    memset(&good_answer.data,0,8);

    can_frame answers[] = {unknown_answer,good_answer};

    std::thread wfmaa_thread(wait_for_msg_and_answer,
                expected_petition,answers,2,true,&fixture_listening);
    test_sleep(); //this delay is so the thread can start TODO, use a lock until the socket is created
    MotorError output;
    EXPECT_NO_THROW(output = odrv.get_motor_error());
    
    if (wfmaa_thread.joinable()){ 
        wfmaa_thread.join();
    }
    can_frame_comparator(expected_petition, msg_buffer[0]);
    can_frame_comparator(unknown_answer, msg_buffer[1]);
    can_frame_comparator(expected_petition, msg_buffer[2]);
    can_frame_comparator(good_answer, msg_buffer[3]);
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

    std::thread wfmaa_thread(wait_for_msg_and_answer,
                expected_petition,&given_answer,1,false,&fixture_listening);
    test_sleep(); //this delay is so the thread can start TODO, use a lock until the socket is created
    MotorError output;
    output = odrv.get_motor_error();
    
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

TEST_F(commandsTest,periodic_pos_estimate_happy_case){
    can_frame heartbeat_msg;
    heartbeat_msg.can_id = (4<<5)|0x009; //4 is odrv id, 9 is cmd_id
    heartbeat_msg.len = 8;
    memset(&heartbeat_msg.data,0,8);
    int talk_soc = create_socket();
    send_to_socket(talk_soc,heartbeat_msg);
    test_sleep();
    EXPECT_TRUE(0.0 == odrv.last_encoder_est.pos_estimate);
    EXPECT_TRUE(0.0 == odrv.last_encoder_est.vel_estimate);
}

TEST_F(commandsTest,periodic_pos_estimate_happy_case_other_value){
    can_frame heartbeat_msg;
    heartbeat_msg.can_id = (4<<5)|0x009; //4 is odrv id, 9 is cmd_id
    heartbeat_msg.len = 8;
    memset(&heartbeat_msg.data,0,8);
    heartbeat_msg.data[2] = 0x80;
    heartbeat_msg.data[3] = 0x3f; // pos_estimate is 1.0
    heartbeat_msg.data[7] = 0x40; // vel_estimate is 2.0
    int talk_soc = create_socket();
    send_to_socket(talk_soc,heartbeat_msg);
    test_sleep();
    EXPECT_TRUE(1.0 == odrv.last_encoder_est.pos_estimate);
    EXPECT_TRUE(2.0 == odrv.last_encoder_est.vel_estimate);
}

TEST_F(commandsTest,periodic_pos_estimate_happy_case_50_values){
    can_frame pos_estimate_msg;
    pos_estimate_msg.can_id = (4<<5)|0x009; //4 is odrv id, 9 is cmd_id
    pos_estimate_msg.len = 8;
    memset(&pos_estimate_msg.data,0,8);
    int talk_soc = create_socket();
    float pos, vel;
    for (float value = 0.0; value < 50.0; value+=1.0){
        pos = value;
        vel = 100.0 - value;
        memcpy(&pos_estimate_msg.data,&pos,4);
        memcpy(&pos_estimate_msg.data[4],&vel,4);
        send_to_socket(talk_soc,pos_estimate_msg);
        test_sleep();
        EXPECT_TRUE(pos == odrv.last_encoder_est.pos_estimate);
        EXPECT_TRUE(vel == odrv.last_encoder_est.vel_estimate);
    }
    fixture_listening = false;
    odrv.listening = false;
}

TEST_F(commandsTest, get_motor_error_waits_for_motor_error_msg){

    can_frame pos_estimate_msg;
    pos_estimate_msg.can_id = (4<<5)|0x009; //4 is odrv id, 9 is cmd_id
    pos_estimate_msg.len = 8;
    memset(&pos_estimate_msg.data,0,8);
    int talk_soc = create_socket();

    can_frame heartbeat_msg;
    heartbeat_msg.can_id = (4<<5)|0x001; //4 is odrv id, 1 is cmd_id
    heartbeat_msg.len = 8;
    memset(&heartbeat_msg.data,0,8);
    heartbeat_msg.data[4] = 1;
    
    can_frame expected_petition;
    expected_petition.can_id = (4<<5)|0x003; //4 is odrv id, 3 is cmd_id
    expected_petition.can_id |= CAN_RTR_FLAG;
    expected_petition.len = 0;

    can_frame given_answer[3];
    given_answer[2].can_id = (4<<5)|0x003; //4 is odrv id, 3 is cmd_id
    given_answer[2].len = 8;
    given_answer[2].data[0] = 0;    //MotorError.NONE;
    memset(&given_answer[2].data,0,8);
    given_answer[0] = pos_estimate_msg;
    given_answer[1] = heartbeat_msg;

    std::thread wfmaa_thread(wait_for_msg_and_answer,
                expected_petition,given_answer, 3, false, &fixture_listening);
    test_sleep(); //this delay is so the thread can start TODO, use a lock until the socket is created
    MotorError output = odrv.get_motor_error();
    if (wfmaa_thread.joinable()){
        wfmaa_thread.join();
    }
   
    
    EXPECT_TRUE(MotorError::NONE == output);
}



int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
