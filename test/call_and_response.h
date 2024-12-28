#include <vector>
#include <string>
#include <iostream>
#include <thread>


#include "odrive_can_cpp.hpp"
#include "odrive_enums.hpp"
#include "test_utils.hpp"


enum class GetFunction{
    MOTOR_ERROR,
    ENCODER_ERROR,
    ENCODER_ESTIMATE,
    ENCODER_COUNTS,
    IQ,
    CONT_ERROR
};

struct testParams
{
    GetFunction func;
    int ans_size;
    // std::vector<uint64_t> bad_msgs;
    std::vector<uint64_t> good_msgs;
};


class callAndResponseCmds : public testing::TestWithParam<testParams>{
    protected:
    void SetUp(){
        listening_thread = std::thread(listener,&fixture_buffer[0],&fixture_listening,&listener_started);
        while (!listener_started){}
    }
    void TearDown(){
        fixture_listening = false;
        odrv.listening = false;
        if (listening_thread.joinable()){
            listening_thread.join();
        }
    }
    

    can_frame fixture_buffer[BUFFER_SIZE];
    bool fixture_listening = true;
    bool listener_started = false;
    std::thread listening_thread;
    odrive_can::OdriveCan odrv;

    //params
    GetFunction func;
    int ans_size;
    int n_of_good_msgs;
    std::vector<uint64_t> good_msgs;

    //containers
    can_frame expected;
    can_frame answers[100];
    can_frame given_answer;

    void param_init(){
        func = GetParam().func;
        ans_size = GetParam().ans_size;
        good_msgs = GetParam().good_msgs;
        n_of_good_msgs = good_msgs.size();
        
        expected.can_id = CAN_RTR_FLAG|(4<<5);
        expected.len = 0;
        given_answer.can_id = (4<<5);
        given_answer.len = ans_size;
    }
    callAndResponseCmds():  odrv("vcan0",4U){} //the id has to be written as litteral otherwise the first test gets a random number
};

TEST_P(callAndResponseCmds,happy_case){
    param_init();
    
    for (uint64_t good_msg: good_msgs){
        switch (func)
        {
        case GetFunction::ENCODER_ERROR:
            /* code */
            expected.can_id |= 4; //cmd_id::ENCODER_ERROR
            given_answer.can_id |= 4;
            memcpy(given_answer.data,&good_msg,ans_size);
            answers[0] = given_answer;
            EncoderError answer;
            break;
        
        default:
            break;
        }
        std::thread wfmaa_thread(wait_for_msg_and_answer,
                    expected,answers, 1, false, &fixture_listening);
        test_sleep();
        EncoderError actual_EncoderError;
        EncoderError expected_EncoderError;
        
        switch (func)
        {
        case GetFunction::ENCODER_ERROR:
            /* code */
            actual_EncoderError = odrv.get_encoder_error();
            memcpy(&expected_EncoderError,&good_msg,ans_size);
            EXPECT_EQ(actual_EncoderError,expected_EncoderError);
            break;
        
        default:
            break;
        }
        if (wfmaa_thread.joinable()){
            wfmaa_thread.join();
        }
    }
}

TEST_P(callAndResponseCmds,bad_size_ignored){
    param_init();

    switch (func){
        case GetFunction::ENCODER_ERROR:
            expected.can_id |= 4; //cmd_id::ENCODER_ERROR
            given_answer.can_id |= 4;
            break;
        default:
            break;
    }
    //first message is wrong size, second message is good size
    memcpy(given_answer.data,&good_msgs[1],ans_size);
    answers[0] = given_answer;
    answers[1] = answers[0];
    memcpy(answers[1].data,&good_msgs[0],ans_size);
    answers[0].len--;
    std::thread wfmaa_thread(wait_for_msg_and_answer,
                    expected,answers, 2, true, &fixture_listening);
    test_sleep();
    EncoderError actual;
    EncoderError expected;
    switch (func){
        case GetFunction::ENCODER_ERROR:
            actual = odrv.get_encoder_error();
            
            memcpy(&expected,&good_msgs[0],ans_size);
            EXPECT_EQ(expected,actual);
        default:
            break;
    }
    fixture_listening = false;
    if (wfmaa_thread.joinable()){
            wfmaa_thread.join();
    }
}

testParams encoder_params = {GetFunction::ENCODER_ERROR,      //GetFunction
                             4,                               //answer size
                             {0,1,2,4,8,0x10,0x20,0x40,0x80}}; //Good messages};                              //number of good messages


INSTANTIATE_TEST_SUITE_P(Values,callAndResponseCmds,
    testing::Values(
        encoder_params)
);
