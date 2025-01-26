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

class messenger_test: public Messenger
{
private:
    /* data */
public:
    messenger_test(const std::string& interface, can_filter filter)
        :Messenger(interface, filter){}
    void callback() override{

    }
};
TEST (msn_classTests, constructor){
    can_filter empty_filter;
    EXPECT_NO_THROW(messenger_test msn("vcan0",empty_filter));
}


