#include "Messenger.hpp"


Messenger::Messenger(const std::string& interface, can_filter filter):
    s_interface_name(interface),
    cf_filter(filter){
        b_listening = true;
}

Messenger::~Messenger(){

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