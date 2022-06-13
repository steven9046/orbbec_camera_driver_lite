## v2.0.1 @2022.6.10 ##
* 1. 添加ZMQ 构建多节点系统
    * a. 把zmq的头文件可库文件加进来了
    * b. 在main函数做了一个pub，但是会卡在那里不动，不知道为什么
    * c. 不知道怎么就好使了又
* 2. 添加 flatbuffers 序列化
    * a. 创建一个简单的shema,并使用flatc构建头文件(注意命名空间问题)
    * b. 
* 3. 把 ZMQ 部分改成和 robox 相同的面向对象形式
    * a. munmap_chunk(): invalid pointer 
        robox 里用的 zmq::message_t msg(buffer, serial_size, zmq_msg_buffer_free, nullptr);
        static void zmq_msg_buffer_free(void* data, void* hint) { delete[] static_cast<uint8_t*>(data); }
        应该是一个释放内存的函数，但是 delete 要和 new 配套使用，如果 delete 了不是 new 出来的东西就会报这个错误
        我们这里的message不是new出来的，肯定会报错，看了一下 message_t 的构造函数，有直接给数据就可以的，换了一下不报错了