#ifndef ODRIVE_CAN
#define ODRIVE_CAN

#include <stdlib.h>


#include <string>
#include <linux/can.h>
namespace odrive_can{

class OdriveCan
{
private:
    /* data */
    int socket_;
public:
    OdriveCan(const std::string& interface, uint32_t filterId);
    ~OdriveCan();
    int sendMessage(const can_frame& frame);
    int receiveMessage(can_frame& frame);
};

}


#endif
