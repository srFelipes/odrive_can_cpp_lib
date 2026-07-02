
#include <stdlib.h>
#include <thread>

#include <string>
#include <linux/can.h>
#include <atomic>
#include <mutex>

#define MSN

typedef enum{
    NOT_ASKING,
    SENDING_REQUEST,
    WAITING_RESPONSE,
}ask_state_t;

class Messenger
{
    private:
        const std::string s_interface_name;
        can_filter cf_filter;
        int i_listening_socket;
        int i_talking_socket;            
        std::atomic<bool> b_listening;
        std::atomic<bool> b_thread_started;
        std::atomic<ask_state_t> ask_state;
        can_frame cf_waiting_for;
        std::mutex listening_mutex;
    protected:
        virtual void callback() = 0;
        std::thread th_listening_thread;
        void listening_routine();
        can_frame last_frame;
        bool listening_started;
        bool thread_start();
        void thread_kill();
        void process_frame(can_frame& input_frame);
    public:
        Messenger(const std::string& interface, can_filter filter);
        ~Messenger();
        int send(can_frame frame);
        bool ask(can_frame &command, int timeout=200);
        bool is_listening();
};
