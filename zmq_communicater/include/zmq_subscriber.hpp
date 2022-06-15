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
 * @param [in] class T  待嵌入类
 * @param [in] class M  c++消息类型
 * @param [in] class Mf flatbuffers消息类型(反序列化时用到)
 */
template <class T, class M, class Mf>
class ZMQSubscriber : public ZMQBase {
 public:
  ZMQSubscriber<T, M, Mf>();
  ~ZMQSubscriber<T, M, Mf>();
  // void init_subscriber(const std::string topic_name, void (T::*fp)(const std::shared_ptr<M const>&), T* obj);
  void init_subscriber(const std::string topic_name, void (T::*fp)(const M*), T* obj);
  void subscriber_connect();
  void subscriber_parse_message();
  // void deserialization(const M&);
  void shutdown();

  std::unique_ptr<zmq::socket_t> sub_;
  std::shared_ptr<std::thread> execution_thread_;
  bool listen_flag_;
 private:
  // 函数形式： 接收 一个指针的引用(M为某种消息这里就是消息指针的引用作为形参) 返回值为空
  std::function<void(const M*)> fp_;
  // std::function<std::unique_ptr<M>(const zmq::message_t&)> unpack_;
};

  template <class T, class M, class Mf>
  ZMQSubscriber<T, M, Mf>::ZMQSubscriber(){
    printf("constructing ZMQSubscriber...\n");
    sub_.reset(new zmq::socket_t(ctx_, ZMQ_SUB)); // 智能指针不需要手动delete
  }

  template <class T, class M, class Mf>
  ZMQSubscriber<T, M, Mf>::~ZMQSubscriber() {
    printf("destorying ZMQSubscriber...\n");
    shutdown();
    execution_thread_->join();
  }

  /**
   * @brief 初始化订阅器
   * @param [in]  topic_name  话题名
   * @param [in]  fp          回调函数指针
   * @param [in]  obj         类指针
   * 1. 绑定回调函数
   * 2. 查找端口号，连接端口
   * 3. 消息回调线程 
   */
  template <class T, class M, class Mf>
  void ZMQSubscriber<T, M, Mf>::init_subscriber(const std::string topic_name, void (T::*fp)(const M*), T* obj) {
    //                   类成员函数指针 类指针 参数(这里只有一个参数)
    this->fp_ = std::bind(fp, obj, std::placeholders::_1);
    topic_name_ = topic_name;
    std::string socket_address = get_socket_str_from_topic_name(topic_name);
    socket_address_ = socket_address;
    listen_flag_ = true;
    std::cout << "Init ZMQ Subscriber - " << topic_name << "  -  " << socket_address_ << std::endl;
    subscriber_connect();
    execution_thread_ = std::make_shared<std::thread>(&ZMQSubscriber<T, M, Mf>::subscriber_parse_message, this);
    // subscriber_parse_message(); // for test once
  }

  /**
   * @brief 解析消息
   * 1. 反序列化，用底层函数 GetRoot 可以在事先不知道类别时就反序列化，不用调用某消息专用的反序列化函数
   * 2. 执行回调
   */
  template <class T, class M, class Mf>
  void ZMQSubscriber<T, M, Mf>::subscriber_parse_message() {
    printf("Starting execution thread.\n");
    // zmq::message_t zmq_msg;
    // typename M::Ptr ptrmsg(new M());
    bool is_message_received = false;
    while (listen_flag_) {
      // printf("listening ...\n");
      zmq::message_t zmq_msg;
      sub_->recv(&zmq_msg);
      is_message_received = true;
      // 消息接收到以后进行回调
      if (is_message_received) {
        // 反序列化
        const Mf* flatbuffers_message = flatbuffers::GetRoot<Mf>(zmq_msg.data());
        M* c_message = flatbuffers_message->UnPack();
        // std::cout << "flatbuffers_message ：" << flatbuffers_message->fx() << std::endl;
        // std::cout << "c_message ：" << c_message->fx << std::endl;
        fp_(c_message);
      }  // has timeout of REQUEST_TIMEOUT variable
      is_message_received = false;
    }
    printf("Execution thread finished!\n");
  }

  template <class T, class M, class Mf>
  void ZMQSubscriber<T, M, Mf>::subscriber_connect() {
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

  // template <class T, class M, class Mf>
  // void ZMQSubscriber<T, M, Mf>::deserialization(const M&){
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

  /**
   * @brief 修改监听flag，停止监听线程
   */
  template <class T, class M, class Mf>
  void ZMQSubscriber<T, M, Mf>::shutdown() {
    printf("shutting down!\n");
    listen_flag_ = false;
  }

}  // namespace zmq_communicater