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
 */

class ZMQPublisher : public ZMQBase {
 public:
  ZMQPublisher();
  ~ZMQPublisher();
  std::unique_ptr<zmq::socket_t> pub_;

  void init_publisher(std::string topic_name);
  // template <typename T>
  void publish(zmq::message_t& message);
};

ZMQPublisher::ZMQPublisher() {}
ZMQPublisher::~ZMQPublisher() {}

/**
 * @brief 
 * @param [in] topic_name   话题名
 * 1. 根据话题名从表里查找到端口号
 * 2. 创建套接字，绑定端口
 */
void ZMQPublisher::init_publisher(std::string topic_name) {
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

/**
 * @brief 模板不能分离编译
 * @param [in] message   待发送消息
 * 这里需要传入的是序列化好的消息，就是zmq消息，不用使用模板
 */
// template <typename T>
void ZMQPublisher::publish(zmq::message_t& message) {
  static int i = 0;
  if (pub_) {
    // std::cout << message.size() << std::endl;
    printf("Publishing count [ %d ]\n", i);
    pub_->send(message);
    i++;
  } else {
    printf("Cannot publish before initialization of the publisher.\n");
  }
}

}  // namespace zmq_communicater