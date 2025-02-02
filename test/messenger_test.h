#include <vector>
#include <string>
#include <iostream>
#include <thread>

#include "Messenger.hpp"
#include "test_utils.hpp"

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
public:
    MessengerDummy(const std::string& interface, can_filter filter)
        :Messenger(interface, filter){}
    can_frame dummy_last_msg;
    void callback() override{
        dummy_last_msg = last_frame;
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
