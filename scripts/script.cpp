#include "odrive_can_cpp.hpp"
#include <iostream>

int main(){
    odrive_can::OdriveCan odrv = odrive_can::OdriveCan("vcan0",0x69);
    can_frame frame;
    std::string data;
    for (int i=0; i<10; i++){
        
        odrv.receive_message(frame);
        std::cout << "message was" << std::endl;
        data = "";
        for (int j = 0; j < frame.can_dlc; ++j) {
            std::cout << " " << std::hex << static_cast<int>(frame.data[j]);
        }
        std::cout << std::endl;
    }
}