// for general
#include <iostream>
#include <zmq_subscriber.hpp>
#include <unistd.h>
// for flatbuffers
#include <camera_generated.h>


/**
 * @brief 对使用 ZMQSubscriber 的类的要求
 * 1. 根据需要的消息类型实现相应的unpack函数，作为参数传给 ZMQSubscriber
 * 2. 实现回调函数，作为参数传给 ZMQSubscriber
 * 
 */
class Test{
public:
    Test();
    ~Test();
    void callback(const std::unique_ptr<Message::CameraT>& message);
    std::unique_ptr<Message::CameraT> unpack(const zmq::message_t& zmq_msg);
};

Test::Test(){}

Test::~Test(){}

void Test::callback(const std::unique_ptr<Message::CameraT>& message){
    std::cout << "callback..." << std::endl;
    std::cout << "deserialized name : " <<message->name << std::endl;
    std::cout << "deserialized fx: " <<message->fx << std::endl;
    std::cout << "deserialized fy: " <<message->fy << std::endl;
    std::cout << "deserialized cx: " <<message->cx << std::endl;
    std::cout << "deserialized cy: " <<message->cy << std::endl;
    // auto data = (char*)zmq_msg.data();
}

std::unique_ptr<Message::CameraT> Test::unpack(const zmq::message_t& zmq_msg){
    auto data = zmq_msg.data();
    // 反序列化，这个默认传入参数是void*
    std::unique_ptr<Message::CameraT> cameraT = Message::UnPackCamera(data);
    return cameraT;
}   

/**
 * @brief 测试程序
 * 1. 需要实例化一个 ZMQSubscriber
 * 2. 加载话题--端口配置文件
 * 3. 初始化订阅器
 */
int main(){
    printf("Testing ZMQSubscriber...\n");
    zmq_communicater::ZMQSubscriber<Test, Message::CameraT> sub_;
    sub_.read_zmq_topic_to_port_file();
    std::string topic_name = "topic_1";
    Test T;
    sub_.init_subscriber(topic_name, &Test::callback, &Test::unpack, &T);
    // while(1){
    //     // printf("listening ...\n");
    //     std::string test_message = "testing";
    //     // sub_.publish(test_message);
    // }
    sleep(10);
    return 0;
}