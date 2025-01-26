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


