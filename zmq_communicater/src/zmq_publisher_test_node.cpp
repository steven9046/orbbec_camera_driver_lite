// for general
#include <iostream>
#include <zmq_publisher.hpp>
#include <unistd.h> // sleep

int main(){
    printf("Testing ZMQPublisher...\n");
    zmq_communicater::ZMQPublisher pub_;
    pub_.read_zmq_topic_to_port_file();
    std::string topic_name = "topic_1";
    pub_.init_publisher(topic_name);
    while(1){
        std::string test_message = "testing";
        pub_.publish(test_message);
        sleep(1);
    }
    return 0;
}