// for general
#include <iostream>
#include <zmq_subscriber.hpp>
#include <unistd.h>

class TestMessage{
public:
    int a;
    typedef std::shared_ptr<TestMessage const> Ptr;
};

class Test{
public:
    Test();
    ~Test();
    void callback(const std::shared_ptr<TestMessage const>& message);
};

Test::Test(){}

Test::~Test(){}

void Test::callback(const std::shared_ptr<TestMessage const>& message){
    std::cout << "callback..." << std::endl;
}

int main(){
    printf("Testing ZMQSubscriber...\n");
    zmq_communicater::ZMQSubscriber<Test, TestMessage> sub_;
    sub_.read_zmq_topic_to_port_file();
    std::string topic_name = "topic_1";
    Test T;

    sub_.init_subscriber(topic_name, &Test::callback, &T);
    // while(1){
    //     // printf("listening ...\n");
    //     std::string test_message = "testing";
    //     // sub_.publish(test_message);
    // }
    sleep(10);
    return 0;
}