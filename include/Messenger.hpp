
#include <stdlib.h>
#include <thread>

#include <string>
#include <linux/can.h>

#define MSN
class Messenger
{
    private:
        const std::string s_interface_name;
        can_filter cf_filter;
        int i_listening_socket;            
        std::thread th_listening_thread;
        bool b_listening;
    protected:
        virtual void callback() = 0;
        void listening();
        can_frame last_frame;
        bool listening_started;
    public:
        Messenger(const std::string& interface, can_filter filter);
        ~Messenger();
        int send(can_frame frame);
        int ask(can_frame command);
        bool is_listening();
        int stop();
        int restart();
};