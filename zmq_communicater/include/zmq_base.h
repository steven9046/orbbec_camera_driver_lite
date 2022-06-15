// for general
#include <iostream>
#include <memory>
#include <map>
// for zmq
#include "libzmq/zmq.hpp"

namespace zmq_communicater{

/**
 * @brief 每个pub和sub必要的成员
 * 1. context 所有派生都是一样的
 * 2. static_topic_to_port_map 所有派生都是一样的，端口表
 * 3. 每个派生不一样的
 */
class ZMQBase {
public:
    ZMQBase();
    virtual ~ZMQBase();
    static zmq::context_t ctx_;
    static std::map<std::string, std::string> static_topic_to_port_map;
    static std::string get_socket_str_from_topic_name(std::string topic_name);
    void read_zmq_topic_to_port_file(); // 其实只进行了一次
    // 
    std::string node_name_;
    std::string topic_name_;
    std::string socket_address_;
};

}