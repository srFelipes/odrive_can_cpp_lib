#include "Messenger.hpp"
#include <time.h>
#include <iostream>
void Messenger::listening_routine(){
    while (b_listening){
        can_frame new_frame;
        last_frame.can_id = 4<<5;
        last_frame.len = 0;
        // last_frame.store(new_frame,std::memory_order_release);
        callback();
    }
}

Messenger::Messenger(const std::string& interface, can_filter filter):
    s_interface_name(interface),
    cf_filter(filter){
        b_listening = false;
    }
Messenger::~Messenger(){
    thread_kill();
}
bool Messenger::thread_start(){
    b_listening = true;
    th_listening_thread = std::thread(&Messenger::listening_routine, this);
    return true;
}
void Messenger::thread_kill(){
    if (th_listening_thread.joinable()){
        b_listening = false;
        th_listening_thread.join();
    }
}
int Messenger::send(can_frame frame){
    return -1;
}
int Messenger::ask(can_frame command){
    return -1;
}
bool Messenger::is_listening(){
    return b_listening;
}
int Messenger::stop(){
    b_listening = false;
    return -1;
}
int Messenger::restart(){
    b_listening = true;
    return -1;
}