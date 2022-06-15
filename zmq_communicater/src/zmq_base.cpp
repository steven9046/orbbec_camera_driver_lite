#include <zmq_base.h>
#define LOCALHOST "tcp://127.0.0.1:"
#define ZMQ_PORT_START_NUM 5000

namespace zmq_communicater{

// 静态变量需要先声明一下
std::map<std::string, std::string> ZMQBase::static_topic_to_port_map;
zmq::context_t ZMQBase::ctx_(1);// 直接初始化了

ZMQBase::ZMQBase(){}
ZMQBase::~ZMQBase(){}

// 从静态变量里得到套接字号
std::string ZMQBase::get_socket_str_from_topic_name(std::string topic_name) {

  std::cout << "Translating..." << std::endl;
  std::cout << "Topic: " << topic_name;
  std::map<std::string, std::string>::iterator it;
  std::string ans;

//   if (topic_name.at(0) != FIRST_CHAR_OF_TOPIC) {
//     topic_name.insert(0, "/");
//   }

//      for (it = static_topic_to_port_map.begin(); it != static_topic_to_port_map.end(); it++) // print whole file
//      {
//          TEMI_LOG(fatal) << it->first  // string (key)
//                    << " : "
//                    << it->second   // string's value
//                    << std::endl ;
//      }

  it = static_topic_to_port_map.find(topic_name);
  if (it != static_topic_to_port_map.end()) {
    ans = it->second;
  } else {
    // TEMI_FATAL(EVNT_GENERIC_TOPIC_NOT_FOUND) << "Topic name: " << topic_name;
    throw "Topic name not found! Add topic to file - " + topic_name;
  }
  std::cout << "     Port: " << ans << std::endl;
  return ans;
}

// 从文件中读取话题并生成相应的套接字号,我们这里可以先设置成一个固定的
void ZMQBase::read_zmq_topic_to_port_file() {
  if (!static_topic_to_port_map.empty()) return;  // need to read file only once
  printf("loading port file...\n");
  // TEMI_LOG(info ) << std::endl << "Reading zmq topic to port file";

//   std::string home_dir = get_user_home_folder();

//   std::string file_path = home_dir + ZMQ_PORT_TO_TOPIC_FILE_NAME;
//   std::ifstream zmq_port_file_object(file_path);

//   if (!zmq_port_file_object.is_open()) {
//     throw "Cannot open input file  - " + file_path;
//   }
//   std::string line_str;
//   int line_number = 0;
//   std::size_t start_pos;
//   std::size_t end_pos;
//   std::string topic_name;

//   while (std::getline(zmq_port_file_object, line_str)) {
//     line_number++;
//     if (line_str.find("#define") == std::string::npos)  // ignore irrelevant lines
//     {
//       continue;
//     }
//     if (line_str.find("/") == std::string::npos)  // must have "/" in topic name!
//     {
//       continue;
//     }

//     start_pos = line_str.find_first_of("/");  // first position of "/" to get beginning of topic
//     end_pos = line_str.find_last_of('"');     // last position to ignore comments
//     topic_name = line_str.substr(start_pos, end_pos - start_pos);

//     // std::cout << "topic name: " << topic_name << " - " << line_number << std::endl;
    for(int line_number = 0; line_number < 1; line_number ++){

        std::string topic_name = "topic_1";
        std::string socket_address = LOCALHOST + std::to_string(ZMQ_PORT_START_NUM + line_number);
        std::pair<std::map<std::string, std::string>::iterator, bool> map_ans =
            static_topic_to_port_map.insert(std::pair<std::string, std::string>(topic_name, socket_address));
    }
//     if (map_ans.second == false) {
//       // TEMI_FATAL(EVNT_GENERIC_ZMQ_PORTS_FILE_INSERT_ERROR) << " In line: " << line_str << " - check for doubles!";
//       throw std::invalid_argument("Topic Already Exists! - " + line_str);
//     }
//   }
  // TEMI_LOG( info ) << "Ports Read Success!!" << std::endl << std::endl;
}

}