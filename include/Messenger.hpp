
#include <stdlib.h>
#include <thread>

#include <string>
#include <linux/can.h>
#include <atomic>

#define MSN
class Messenger
{
    private:
        const std::string s_interface_name;
        can_filter cf_filter;
        int i_listening_socket;            
        std::atomic<bool> b_listening;
        std::atomic<bool> b_thread_started;
    protected:
        virtual void callback() = 0;
        std::thread th_listening_thread;
        void listening_routine();
        can_frame last_frame;
        bool listening_started;
        bool thread_start();
        void thread_kill();
    public:
        Messenger(const std::string& interface, can_filter filter);
        ~Messenger();
        int send(can_frame frame);
        int ask(can_frame command);
        bool is_listening();
        int stop();
        int restart();
};