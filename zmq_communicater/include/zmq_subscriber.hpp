// for general
// #include <boost/function.hpp>
#include <iostream>
#include <memory>
#include <thread>
#include <functional>

// for zmq
#include "libzmq/zmq.hpp"
#include "zmq_base.h"

// for flatbuffers
#include <camera_generated.h> // 反序列化用到了

namespace zmq_communicater {

/**
 * @brief
 * contex 在 base 里已经有了，这里只需要设置套接字就行了
 *
 */
//    ZMQSubscriber 是嵌入到某个class里的
//         传入obj  消息类型
template <class T, class M>
class ZMQSubscriber : public ZMQBase {
 public:
  ZMQSubscriber<T, M>();
  ~ZMQSubscriber<T, M>();
  // void init_subscriber(const std::string topic_name, void (T::*fp)(const std::shared_ptr<M const>&), T* obj);
  void init_subscriber(const std::string topic_name, void (T::*fp)(const std::unique_ptr<M>&), std::unique_ptr<M> (T::*unpack)(const zmq::message_t&), T* obj);
  void subscriber_connect();
  void subscriber_parse_message();
  // void deserialization(const M&);
  void shutdown();

  std::unique_ptr<zmq::socket_t> sub_;
  std::shared_ptr<std::thread> execution_thread_;
  bool listen_flag_;
  // 消息回调函数
 private:
  // 函数形式： 接收 一个指针的引用(M为某种消息这里就是消息指针的引用作为形参) 返回值为空
  std::function<void(const std::unique_ptr<M>&)> fp_;
  std::function<std::unique_ptr<M>(const zmq::message_t&)> unpack_;
};

  template <class T, class M>
  ZMQSubscriber<T, M>::ZMQSubscriber(){
    printf("constructing ZMQSubscriber...\n");
    sub_.reset(new zmq::socket_t(ctx_, ZMQ_SUB)); // 这里new出来的记得delete
  }

  template <class T, class M>
  ZMQSubscriber<T, M>::~ZMQSubscriber() {
    printf("destorying ZMQSubscriber...\n");
    shutdown();
    execution_thread_->join();
  }

  template <class T, class M>  // T的类成员函数指针 &CEventQuery::DoSomething;
  void ZMQSubscriber<T, M>::init_subscriber(const std::string topic_name, void (T::*fp)(const std::unique_ptr<M>&), std::unique_ptr<M> (T::*unpack)(const zmq::message_t&), T* obj) {
    //                   类成员函数指针 类指针 参数(这里只有一个参数)
    this->fp_ = std::bind(fp, obj, std::placeholders::_1);
    this->unpack_ = std::bind(unpack, obj, std::placeholders::_1);
    topic_name_ = topic_name;
    std::string socket_address = get_socket_str_from_topic_name(topic_name);
    socket_address_ = socket_address;
    listen_flag_ = true;
    std::cout << "Init ZMQ Subscriber - " << topic_name << "  -  " << socket_address_ << std::endl;
    subscriber_connect();
    execution_thread_ = std::make_shared<std::thread>(&ZMQSubscriber<T, M>::subscriber_parse_message, this);
    // subscriber_parse_message(); // for test once
  }

  template <class T, class M>
  void ZMQSubscriber<T, M>::subscriber_parse_message() {
    printf("Starting execution thread.\n");
    // zmq::message_t zmq_msg;
    // typename M::Ptr ptrmsg(new M());
    bool is_message_received = false;
    while (listen_flag_) {
      printf("listening ...\n");
      zmq::message_t zmq_msg;
      sub_->recv(&zmq_msg);
      is_message_received = true;

      // 消息接收到以后进行回调
      if (is_message_received) {
        // ros::serialization::IStream s(static_cast<uint8_t*>(zmq_msg.data()), zmq_msg.size());
        // ros::serialization::deserialize(s, *ptrmsg);
            // 这里得用这个函数
        const Message::Camera* x = flatbuffers::GetRoot<Message::Camera>(zmq_msg.data());
        std::cout << "x ：" << x->fx() << std::endl;
        Message::CameraT* y = x->UnPack();
        std::cout << "unserialized_msg ：" << y->fx << std::endl;

        auto unserialized_msg = unpack_(zmq_msg);
        fp_(unserialized_msg);
      }  // has timeout of REQUEST_TIMEOUT variable
      is_message_received = false;
    }
    printf("Execution thread finished!\n");
  }

  template <class T, class M>
  void ZMQSubscriber<T, M>::subscriber_connect() {
    // std::lock_guard<std::mutex> lock(subscriber_connect_mutex_);
    try {
      sub_->setsockopt(ZMQ_SUBSCRIBE, "", 0);
      sub_->setsockopt(ZMQ_LINGER, 0);  // ALWAYS
      sub_->connect(socket_address_.c_str());
    } catch (std::exception& err) {
      std::cout << "******** ERROR INIT TOPIC - " << topic_name_ << "    PORT - " << socket_address_ << "***********" << std::endl;
      exit(1);
    }
  }

  // template <class T, class M>
  // void ZMQSubscriber<T, M>::deserialization(const M&){
  //   auto data = zmq_msg.data();
  //   // 反序列化，这个默认传入参数是void*
  //   std::unique_ptr<Message::CameraT> cameraT = Message::UnPackCamera(data);

  //   std::cout << "deserialized name : " <<cameraT->name << std::endl;
  //   std::cout << "deserialized fx: " <<cameraT->fx << std::endl;
  //   std::cout << "deserialized fy: " <<cameraT->fy << std::endl;
  //   std::cout << "deserialized cx: " <<cameraT->cx << std::endl;
  //   std::cout << "deserialized cy: " <<cameraT->cy << std::endl;
  //   // auto data = (char*)zmq_msg.data();
  //   auto size = zmq_msg.size();
  //   std::cout << "message size : " << size << std::endl;
  // }

  template <class T, class M>
  void ZMQSubscriber<T, M>::shutdown() {
    printf("shutting down!\n");
    listen_flag_ = false;
  }

}  // namespace zmq_communicater