#include <vector>
#include <string>
#include <iostream>
#include <thread>

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