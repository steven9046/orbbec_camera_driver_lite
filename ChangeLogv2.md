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

## v2.2.0 @2022.6.10 ##
* 1. Eigen3 的 CMakeList 写法
    竟然都是大写
    find_package(Eigen3 REQUIRED)
    include_directories (${EIGEN3_INCLUDE_DIRS})
* 2. 驱动部分只读取数据，不生成点云
* 3. 如何在别的节点里找到本节点生成的动态库，findpackage的原理