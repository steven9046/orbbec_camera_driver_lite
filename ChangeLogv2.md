## v2.0.1 @2022.6.10 ##
* 1. 添加ZMQ 构建多节点系统
    * a. 把zmq的头文件可库文件加进来了
    * b. 在main函数做了一个pub，但是会卡在那里不动，不知道为什么
    * c. 不知道怎么就好使了又

* 2. 添加 flatbuffers 序列化
    * a. 创建一个简单的shema,并使用flatc构建头文件(注意命名空间问题)
    * b. 根据头文件里的构造函数来创建序列化消息
         创建 builder
         create 消息
         builder finish
         获取消息的数据区指针及大小
    * c. 原地进行反序列化，测试使用是否正确

* 3. 把 ZMQ 部分改成和 robox 相同的面向对象形式
    * a. munmap_chunk(): invalid pointer 
        robox 里用的 zmq::message_t msg(buffer, serial_size, zmq_msg_buffer_free, nullptr);
        static void zmq_msg_buffer_free(void* data, void* hint) { delete[] static_cast<uint8_t*>(data); }
        应该是一个释放内存的函数，但是 delete 要和 new 配套使用，如果 delete 了不是 new 出来的东西就会报这个错误
        我们这里的message不是new出来的，肯定会报错，看了一下 message_t 的构造函数，有直接给数据就可以的，换了一下不报错了
    * b. 在主循环里发送一次message以后就需要重新构建message再发送了，要不然这个message就无了，像个消耗品一样


* 4. 添加一个订阅端
    * a. 和发布端差不多，谁bind谁connect没有硬性要求
    * b. 需要设置 setsockopt ,否则无法接收消息
    * c. 反序列化 zmqmessage -> flatbuffer
         这里就是读取 一块数据, 注意 zmqmessage.data 返回的是 void* ,转成flatbuffers时需要指定类型
    * d. 如果读取 flatbuffers 结构体里的数据
         直接返回的 name() 不是string， 而是flatbuffers::string
         头文件里的那些 unpack pack 函数是用来干什么的？
         unpack 是把 flatbuffer 结构体反序列化为 c++ 的对象，反序列化以后就按照 c++ 对象使用就可以了
    * e. unpack 传入数据区指针是 void*， 之前一直段错误是因为把这个指针转成 uint8_t 了，这是不行的

## v2.1.0 @2022.6.10 ##
* 1. Eigen3 的 CMakeList 写法
    竟然都是大写
    find_package(Eigen3 REQUIRED)
    include_directories (${EIGEN3_INCLUDE_DIRS})
* 2. 驱动部分只读取数据，不生成点云
* 3. 如何在别的节点里找到本节点生成的动态库，findpackage的原理

## v2.1.1 @2022.6.10 ##
**编写通信功能包**
* 1. 编写 base
    * a. topic to port 先写个固定的充数 , base 的主要功能就是这个
    * b. 把根据 topic 找 port 实现了
    * c. 所有套接字共用一个 contex

* 2. 编写publisher
    * a. 初始化时接收参数 topic ，转化成 port
    * b. 有模板函数，所以写成了hpp

* 3. 编写subscriber
    * a. 因为要接收不同消息，所以采用模板编程，不能分离编译，所以都写在头文件里
    * b. 初始化需要传入参数为 topic 
         根据topic去map里找到相应的端口号,保存到成员变量里
    * c. 开启解析线程(这里为什么要用多线程？)
         感觉并不需要，直接在主线程里进行 Loop 就可以了
    * d. 把boost里的都替换成了std里的
    * e. std::thread 用的实现到了 pthread， CMakeList里加一个
         target_link_libraries(project_name
                                pthread)
    * f. 改成hpp
    * g. 析构时需要把线程join了
**测试节点可以通信，接下来需要进行消息封装**


