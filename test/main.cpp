#include <gtest/gtest.h>
#include "odrive_can_cpp.hpp"

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

int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv); 
    return RUN_ALL_TESTS();
}