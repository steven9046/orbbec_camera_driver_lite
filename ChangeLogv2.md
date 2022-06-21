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

## v2.1.2 @2022.6.15 ##
* 1. 添加序列化库flatbuffers进入CMakeList
    * a. 如何得到上层目录
* 2. 给 pub 添加序列化
* 3. 给 sub 添加反序列化
    * a. sub zmq_message_t 没有定义 ptr, 所以把用到 <M> 的地方都改成了引用
    * b. 在接收数据时并没有进行上锁等操作，可能不安全
    * c. 不同消息类型的反序列化函数不一样，这里还需要用函数模板传进来一个函数
         这里就体现出了用flatbuffers的不方便的地方，就是反序列化不方便，ros的message包不用考虑消息类型，都用的同一个反序列化函数
         怎么从序列化数据里知道类型?
         直接就用roottype就行了
    * d. 那这样还不如把反序列化放到外部类里来做，反正都是调用外部函数
    * e. 最后通过多添加一个模板参数，实现在内部反序列化

## v2.1.3 @2022.6.16 ##
* 1. 给 orbbec_camera_node 添加 publisher
    * a. 各种头文件包含也太不方便了，得研究一下  CMake catkin 是怎么管理这些东西的
    * b. CMake 的 Install
        1. 设置 CMAKE_INSTALL_PREFIX
        2. 设置 install 选项  
        3. 执行 make install
    * c. install 命令
        1. TARGETS 后是所有自己想安装的东西，后边会根据 targets 的类别自动安装到相应的目录下边
            install(TARGETS zmq_communicater zmq_publisher_test_node zmq_subscriber_test_node
            RUNTIME DESTINATION bin
            LIBRARY DESTINATION lib
            ARCHIVE DESTINATION lib/static)
        2. 如果自己还想安装一些特定的文件可以使用
            install(DIRECTORY include/ DESTINATION include)
            带 / 是安装 include文件夹下的文件到 include  -->  include/*
            不带 / 是安装整个文件夹到 include 下         -->  include/include/*
            https://blog.csdn.net/qq_38410730/article/details/102837401

* 2. 如何使生成的包可以被其他包使用
    * a. 生成 .cmake 文件
        1. install(TARGETS FlatbuffersMessage  
            DESTINATION lib 
            EXPORT flatbuffer_message # 并导出库信息
            )
            这里的 EXPORT 可以导出库信息到一个抽象对象里
        2. install(EXPORT flatbuffer_message 
                FILE flatbuffer_message.cmake
                DESTINATION lib/cmake/flatbuffer_message
                )
            这里根据上一步生成的抽象对象生成.cmake文件

    * b. 生成 Config.cmake 文件
        上一步生成的 .cmake 文件不能被 findpackage 直接找到，
        findpackage 需要先找到 Config.cmake 在通过这个文件找到所有需要的 .cmake (如果本项目还有其他依赖的包，也是在这里设置的)
        1. CMake 也有许多功能模块，需要用哪个模块的时候需要 include()
            include(CMakePackageConfigHelpers)
        2. configure_package_config_file
            这个可以根据 in 文件生成 Config.cmake
        3. 编写 .in 文件
            如果没有第三方依赖只需要包含当前包的.cmake
            include("${CMAKE_CURRENT_LIST_DIR}/flatbuffer_message.cmake")
            如果当前包还依赖其他第三方包，则需要写进来(不确定)
            include(CMakeFindDependencyMacro) 
        4. 这里还可以设置一些 .cmake 里的宏
            例如我们这里就是把头文件路径写在这里了
            set(flatbuffer_message_INCLUDE_DIR "${PACKAGE_PREFIX_DIR}/include")
            (写在CMakeList里的话其他包是无法使用这个宏的)
            (这里包含头文件应该有其他办法，例如 PUBLIC_HEADER 暂时没有尝试)
    * c. 版本号设置
        1. set (FlatbufferMessage_VERSION_MAJOR 1)
           set (FlatbufferMessage_VERSION_MINOR 0)
           这句就可以直接设置版本号了，后边的应该是让第三方引用时可以在cpp里读取版本号

* 3. zmq 找包的问题
        在test项目里试了很久 
     * a. libzmq.so -> libzmq.so.5.1.5          编译时按照这个软链接找
          libzmq.so.5 -> libzmq.so.5.1.5        运行时按照这个软链接找
     * b. 我们遇到的问题是即使是
          把 libzmq.so.5.1.5 放到源码目录里了，并且建立了相应的软链接 libzmq.so libzmq.so.5
          但是运行时还是会有先找到 /usr/local/lib 里的 libzmq.so.5.1.5
          把/usr/local/lib 里的库删了以后才会找到源码里的那个 libzmq.so.5.1.5
          试了一下把名字都改成 libzeromq* ，看一下能不能解决shadow的问题，然而还是找 libzmq.so.5
          这说明 libzmq.so.5.1.5 里边就写死了要去找 libzmq.so.5
          看来只能是修改一下环境变量，让运行时先到自己设定的路径里去找 .so
          加到 LD_LIBRARY_PATH 里了还是会去 /usr/local/lib 里找
          (PATH 是可执行程序查找路径， LD_LIBRARY_PATH 是动态库查找路径)
          这里先把系统的库删除吧

* 4. 最后整理的结构,一共三个包
     libflatbuffers -> FlatbufferMessage
     FlatbufferMessage libzmq -> zmq_communicater 
     FlatbufferMessage zmq_communicater -> orbbec_camera_driver
     zmq 包里 sub 用到了反序列化的函数，已经深度绑定了，所以把 flatbuffer 和 zmq 分开的意义不大了
     那还不如把这两个包合成一个