#include <vector>
#include <string>
#include <iostream>
#include <thread>
#include <chrono>

#include "Messenger.hpp"
#include "test_utils.hpp"

int wait_timeout = 100;

TEST(msn_importingTests,importMessenger){
    #ifdef MSN
    EXPECT_TRUE(true);
    #else 
    EXPECT_TRUE(false);
    #endif
}

class MessengerDummy: public Messenger
{
private:
    /* data */
    int number_of_callbacks;
public:
    void stop(){
        thread_kill();
    }
    void restart(){
        if (!is_listening()){
            thread_start();
        }
    }
    void callback() override{
        number_of_callbacks++;
        dummy_last_msg = last_frame;
    }
    MessengerDummy(const std::string& interface, can_filter filter)
        :Messenger(interface, filter){
            number_of_callbacks = 0;
            thread_start();
        }
    can_frame dummy_last_msg;
    int get_number_of_callbacks(){
        return number_of_callbacks;
    }
    can_frame get_last_msg(){
        return dummy_last_msg;
    }
    ~MessengerDummy(){
        thread_kill();
    }
};
TEST (msn_classTests, constructor){
    can_filter empty_filter;
    EXPECT_NO_THROW(MessengerDummy msn("vcan0",empty_filter));
}

class msn_fixture : public testing::Test{
    public:
        void SetUp(){
            socket = create_socket();
        }
        void TearDown(){
            close(socket);
        }
        MessengerDummy msn;
        can_filter filter = {{4<<5},{0x7e0}};
        int socket;
        msn_fixture(): msn("vcan0",filter){};
};

TEST_F(msn_fixture,start_stop_restart){
    EXPECT_TRUE(msn.is_listening());
    msn.stop();
    EXPECT_FALSE(msn.is_listening());
    msn.restart();
    EXPECT_TRUE(msn.is_listening());
}

TEST_F(msn_fixture, listen_1_msg){
    can_frame expected;
    expected.can_id = 4 << 5;
    expected.len = 0;
    int send_socket = create_socket(); 
    send_to_socket(send_socket, expected);
    int wait_result = wait_for_condition_with_timeout(
                      [this](){return (msn.get_number_of_callbacks() > 0);},
                      wait_timeout);
    EXPECT_TRUE(0 == wait_result);
    EXPECT_TRUE(expected == msn.get_last_msg());
}

TEST_F(msn_fixture, listen_10_msg){
    can_frame expected;
    expected.can_id = 4 << 5;
    expected.len = 1;
    int send_socket = create_socket();
    for (int i = 0; i<10 ; i++){
        expected.data[0] = i;
        send_to_socket(send_socket, expected);
        int wait_result = wait_for_condition_with_timeout(
                        [this, i](){return (msn.get_number_of_callbacks() ==(i+1));},
                        wait_timeout);
        EXPECT_TRUE(0 == wait_result);
        EXPECT_TRUE(expected == msn.get_last_msg());
    }
}

TEST_F(msn_fixture, filter_10msg
){
    can_frame expected, not_expected;
    expected.can_id = 4 << 5;
    expected.len = 1;
    not_expected.can_id = 3 << 5;
    not_expected.len = 1;
    int send_socket = create_socket();
    int last_n_of_calls = 0;
    for (int i = 0; i<10 ; i++){
        expected.data[0] = i;
        not_expected.data[0] = i;
        send_to_socket(send_socket, expected);
        send_to_socket(send_socket, not_expected);
        int wait_result = wait_for_condition_with_timeout(
                        [this, i](){return (msn.get_number_of_callbacks() ==(i+1));},
                        wait_timeout);
        EXPECT_TRUE(0 == wait_result);
        EXPECT_TRUE(expected == msn.get_last_msg());
        EXPECT_FALSE(not_expected == msn.get_last_msg());
    }
}

class msn_with_listener: public msn_fixture{
    public:
        void SetUp(){
            thread_alive = true;
            thread_started = false;
            socket = create_socket();
            fixture_cf_buffer.reserve(2050);
            fixture_thread = std::thread(listener,
                                         &fixture_cf_buffer,
                                         &thread_alive,
                                         &thread_started,
                                         &n_received_msgs);
            while (!thread_started){};
        }
        void TearDown(){
            close(socket);
            thread_alive = false;
            if (fixture_thread.joinable()){
                fixture_thread.join();
            }
        }
        std::vector<can_frame> fixture_cf_buffer;
        std::thread fixture_thread;
        std::atomic <bool> thread_alive;
        std::atomic <bool> thread_started;
        std::atomic <int> n_received_msgs;
}; 

TEST_F(msn_with_listener, send_1_msg){
    can_frame expected;
    expected.len = 1;
    expected.data[0] = 0xfe;
    expected.can_id = 4 << 5;
    int send_result = msn.send(expected);
    int wait_result = wait_for_condition_with_timeout(
        [this](){return (n_received_msgs == 1);},
        wait_timeout);
    EXPECT_TRUE(0 == wait_result);
    EXPECT_TRUE(expected == fixture_cf_buffer[0]);
    EXPECT_TRUE(0 == send_result);    
}

TEST_F(msn_with_listener, ask_1_msg){
    std::atomic <bool> wfmaa_started;
    std::atomic <bool> wfmaa_alive;

    wfmaa_alive = true;
    wfmaa_started = false;
    std::vector<can_frame> messages;

    can_frame petition;
    petition.can_id = (4 << 5) | (20);
    petition.len = 0;

    can_frame expected_answer;
    expected_answer.can_id = petition.can_id;
    expected_answer.len = 1;
    expected_answer.data[0] = 69;

    can_frame petition_for_thread = petition;
    petition_for_thread.can_id |= CAN_RTR_FLAG;

    messages.push_back(petition_for_thread);
    messages.push_back(expected_answer);

    std::thread listen_and_answer_thread(listen_and_answer,
                                         std::ref(messages),
                                         std::ref(wfmaa_started),
                                         std::ref(wfmaa_alive),
                                         false);
    while(!wfmaa_started){};
    EXPECT_TRUE(msn.ask(petition));
    int wait_result = 
        wait_for_condition_with_timeout(
            [this](){return (2 == n_received_msgs);},
            wait_timeout);
    wfmaa_alive = false;
    if (listen_and_answer_thread.joinable()){
        listen_and_answer_thread.join();
    }    
    EXPECT_TRUE(2 == n_received_msgs);
    EXPECT_TRUE(expected_answer == petition);
    EXPECT_TRUE(petition_for_thread == fixture_cf_buffer[0]);
    EXPECT_TRUE(expected_answer == fixture_cf_buffer[1]);
}

TEST_F(msn_with_listener, ask_10_msgs){
    std::atomic <bool> wfmaa_started;
    std::atomic <bool> wfmaa_alive;

    wfmaa_alive = true;
    wfmaa_started = false;
    std::vector<can_frame> messages;

    can_frame petition;
    petition.can_id = (4 << 5) | (20);
    petition.len = 0;

    can_frame expected_answer;
    expected_answer.can_id = petition.can_id;
    expected_answer.len = 1;

    can_frame petition_for_thread = petition;
    petition_for_thread.can_id |= CAN_RTR_FLAG;

    messages.push_back(petition_for_thread);

    for (int i=0; i<10; i++){
        expected_answer.data[0] = i;
        expected_answer.len = 1;
        messages.push_back(expected_answer);
    }
    
    std::thread listen_and_answer_thread(listen_and_answer,
                                         std::ref(messages),
                                         std::ref(wfmaa_started),
                                         std::ref(wfmaa_alive),
                                         true);
    while(!wfmaa_started){};
    int wait_result;
    for (int k=0; k<10; k++){
        EXPECT_TRUE(msn.ask(petition));
        wait_result = 
        wait_for_condition_with_timeout(
            [this, k](){return (2*(k+1) == n_received_msgs);},
            wait_timeout);
        EXPECT_TRUE(0 == wait_result);
        expected_answer.data[0] = k;
        EXPECT_TRUE(expected_answer == petition);

        EXPECT_TRUE(petition_for_thread == fixture_cf_buffer[2*k]);
        EXPECT_TRUE(expected_answer == fixture_cf_buffer[2*k+1]);
        petition.can_id = (4 << 5) | (20);
        petition.len = 0;
    }
    
    wfmaa_alive = false;
    if (listen_and_answer_thread.joinable()){
        listen_and_answer_thread.join();
    }
    
}

TEST_F(msn_with_listener, ask_n_msgs){
    int num_of_messages = 1000;
    std::atomic <bool> wfmaa_started;
    std::atomic <bool> wfmaa_alive;

    wfmaa_alive = true;
    wfmaa_started = false;
    std::vector<can_frame> messages;

    can_frame petition;
    petition.can_id = (4 << 5) | (20);
    petition.len = 0;

    can_frame expected_answer;
    expected_answer.can_id = petition.can_id;
    expected_answer.len = 1;

    can_frame petition_for_thread = petition;
    petition_for_thread.can_id |= CAN_RTR_FLAG;

    messages.reserve(num_of_messages+1);
    messages.push_back(petition_for_thread);

    for (int i=0; i<num_of_messages; i++){
        expected_answer.data[0] = 69;
        expected_answer.len = 1;

        messages.push_back(expected_answer);
    }
    
    std::thread listen_and_answer_thread(listen_and_answer,
                                         std::ref(messages),
                                         std::ref(wfmaa_started),
                                         std::ref(wfmaa_alive),
                                         true);
    while(!wfmaa_started){};
    int wait_result;
    for (int k=0; k<num_of_messages; k++){
        std::cout << "-------------" << std::endl;
        ASSERT_TRUE(msn.ask(petition,1000));
        wait_result = 
        wait_for_condition_with_timeout(
            [this, k](){return (2*(k+1) == n_received_msgs);},
            wait_timeout);
        EXPECT_TRUE(0 == wait_result);
        expected_answer.data[0] = 69;
        EXPECT_TRUE(expected_answer == petition);

        EXPECT_TRUE(petition_for_thread == fixture_cf_buffer[2*k]);
        EXPECT_TRUE(expected_answer == fixture_cf_buffer[2*k+1]);
        petition.can_id = (4 << 5) | (20);
        petition.len = 0;
    }
    
    wfmaa_alive = false;
    if (listen_and_answer_thread.joinable()){
        listen_and_answer_thread.join();
    }
    
}

TEST_F(msn_with_listener, ask_1_msg_timeout){
    std::vector<can_frame> messages;

    can_frame petition;
    petition.can_id = (4 << 5) | (20);
    petition.len = 0;
    can_frame old_petition = petition;
    EXPECT_FALSE(msn.ask(petition));
    int wait_result = 
        wait_for_condition_with_timeout(
            [this](){return (1 == n_received_msgs);},
            wait_timeout);
    EXPECT_TRUE(1 == n_received_msgs);
    EXPECT_TRUE(old_petition == petition);
}

TEST_F(msn_with_listener, ask_1_msg_timeout_different_val){
    std::vector<can_frame> messages;

    std::chrono::steady_clock::now();
    can_frame petition;
    petition.can_id = (4 << 5) | (20);
    petition.len = 0;
    can_frame old_petition = petition;
    auto t1 = std::chrono::steady_clock::now();
    EXPECT_FALSE(msn.ask(petition,10));
    auto t2 = std::chrono::steady_clock::now();
    auto diff = std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1);
    EXPECT_TRUE(diff.count() < 100);
    EXPECT_TRUE(1 == n_received_msgs);
    EXPECT_TRUE(old_petition == petition);
}
