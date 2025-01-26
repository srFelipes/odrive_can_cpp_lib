#include "Messenger.hpp"


Messenger::Messenger(const std::string& interface, can_filter filter):
    s_interface_name(interface),
    cf_filter(filter){
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
    return false;
}
int Messenger::stop(){
    return -1;
}
int Messenger::restart(){
    return -1;
}