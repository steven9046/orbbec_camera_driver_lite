// for general
#include <iostream>
#include <memory>
// for zmq
#include "libzmq/zmq.hpp"
#include "zmq_base.h"

namespace zmq_communicater {

/**
 * @brief
 * contex 在 base 里已经有了，这里只需要设置套接字就行了
 *
 */
class ZMQPublisher : public ZMQBase {
 public:
  ZMQPublisher();
  ~ZMQPublisher();
  std::unique_ptr<zmq::socket_t> pub_;

  void init_publisher(std::string topic_name);
  template <typename T>
  void publish(T& message);
};

ZMQPublisher::ZMQPublisher() {}
ZMQPublisher::~ZMQPublisher() {}

void ZMQPublisher::init_publisher(std::string topic_name) {
  // printf("Initializing ZMQPublisher... address: %s\n",socket_address.c_str());
  std::string socket_address = get_socket_str_from_topic_name(topic_name);
  std::cout << "Initializing ZMQPublisher..." << std::endl;
  std::cout << "Address: " << socket_address << std::endl;
  socket_address_ = socket_address;
  try {
    if (!pub_) {
      pub_.reset(new zmq::socket_t(ctx_, ZMQ_PUB));
    }
    pub_->bind(socket_address_.c_str());
  } catch (std::exception& err) {
    printf("something wrong!\n");
  }
  std::cout << "ZMQPublisher Initialized!" << std::endl;
}

// 类模板不能分离编译
// 这里最好是传入序列化好的数据
template <typename T>  // 只有这一个函数用到了模板，所有不能分成声明和
void ZMQPublisher::publish(T& message) {
  static int i = 0;
  if (pub_) {
    //   uint32_t serial_size = ros::serialization::serializationLength(message);
    //   uint8_t* buffer = new uint8_t[serial_size];

    //   ros::serialization::OStream stream(buffer, serial_size);
    //   ros::serialization::serialize(stream, message);
    // char szBuf[1024] = {0};
    // snprintf(szBuf, sizeof(szBuf), "server i=%d", i);
    // zmq::message_t msg(szBuf, sizeof(szBuf));  //, zmq_msg_buffer_free, nullptr
    std::cout << message.size() << std::endl;
    pub_->send(message);
    //   printf("publishin : %s\n", szBuf);
    i++;
  } else {
    printf("Cannot publish before initialization of the publisher.\n");
  }
}

}  // namespace zmq_communicater