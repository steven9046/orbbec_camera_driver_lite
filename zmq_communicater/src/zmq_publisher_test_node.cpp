// for general
#include <iostream>
#include <zmq_publisher.hpp>
#include <unistd.h> // sleep
// for flatbuffers
#include <camera_generated.h>

int main(){
    printf("Testing ZMQPublisher...\n");
    zmq_communicater::ZMQPublisher pub_;
    pub_.read_zmq_topic_to_port_file();
    std::string topic_name = "topic_1";
    pub_.init_publisher(topic_name);

    // 创建消息并序列化
    flatbuffers::FlatBufferBuilder builder;
    // auto Camera = new Message::Camera; // 默认构造函数被删除了
    // 创建内部格式数据
    auto name = builder.CreateString("Orbbec");
    float fx = 454.343;
    float fy = 454.343;
    float cx = 327.461;
    float cy = 246.672;  
    float k1 = 0.0552932;  
    float k2 = -0.0753816;  
    float k3 = 0.;
    // 创建对象
    auto camera = Message::CreateCamera(builder, name, fx, fy, cx, cy, k1, k2, k3);
    builder.Finish(camera);
    // 获取数据区指针与大小
    uint8_t* buf = builder.GetBufferPointer();
    int size = builder.GetSize();

    while(1){
        // std::string test_message = "testing";
        zmq::message_t msg(buf, size);
        pub_.publish(msg);
        sleep(1);
    }
    return 0;
}