# ros-learning-notes

# 安装前配置
```
// 20.04版本需要
sudo apt-get install ubuntu-pro-client-l10n:amd64
```
s-s
```
export http_proxy=socks5://127.0.0.1:1080
export https_proxy=socks5://127.0.0.1:1080
```
v-2
```
export http_proxy=http://127.0.0.1:10871
export https_proxy=http://127.0.0.1:10871
```

# ubuntu20.04安装ros noetic
## 一、安装ROS neotic步骤

### 第一步：换源（在软件和更新里面）
这一步非常重要，决定着后面安装ROS能否成功的关键一步。
这里，我选择了中科大（https://mirrors.ustc.edu.cn/ubuntu/）。

### 第二步：添加ROS软件源
```
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
```

### 第三步：添加密钥
```
sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
```

### 第四步：更新软件源
```
sudo apt update
```

### 第五步：开始安装ROS
```
sudo apt install ros-noetic-desktop-full
```

### 第六步：设置/更新环境变量
```
echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
```

```
source ~/.bashrc
```

### 第七步：安装ros所需工具
```
sudo apt install python3-rosinstall python3-rosinstall-generator python3-wstool build-essential python3-roslaunch
```

## 二、测试是否安装成功
### 第一步：终端里输入roscore
```
roscore
```

### 第二步：重新打开终端输入
```
rosrun turtlesim turtlesim_node
```

### 第三步：再重新打开终端输入
```
rosrun turtlesim turtle_teleop_key
```
把鼠标放在第三步的界面上，然后通过上下左右方向键就可以控制小海龟的移动了。
至此，ros已经安装完成了。

# ros1 demo（C++）
下面是一个简单的 ROS 1 demo，包括创建一个 Catkin 工作空间、创建一个 ROS 节点以发布一个字符串消息到一个话题上，并创建一个订阅器来接收并打印这个消息。这个 demo 包含两个文件：`CMakeLists.txt` 和 `talker_listener_demo.cpp`。

首先，创建一个 Catkin 工作空间：

1. 打开终端。
2. 运行以下命令，创建一个名为 `catkin_ws` 的 Catkin 工作空间：
    ```bash
    mkdir -p ~/catkin_ws/src
    cd ~/catkin_ws/src
    catkin_init_workspace
    ```

接下来，创建 ROS 节点和订阅器的源文件：

1. 在 `~/catkin_ws/src` 目录下创建一个名为 `talker_listener_demo` 的包，并进入该目录：
    ```bash
    cd ~/catkin_ws/src
    catkin_create_pkg talker_listener_demo roscpp std_msgs
    cd talker_listener_demo
    ```

2. 在 `talker_listener_demo` 包中创建一个名为 `talker_listener_demo.cpp` 的 C++ 源文件，并在其中添加以下代码：

    ```cpp
    #include "ros/ros.h"
    #include "std_msgs/String.h"

    // 回调函数，用于处理接收到的消息
    void chatterCallback(const std_msgs::String::ConstPtr& msg)
    {
      ROS_INFO("I heard: [%s]", msg->data.c_str());
    }

    int main(int argc, char **argv)
    {
      // 初始化 ROS 节点
      ros::init(argc, argv, "talker_listener_demo");

      // 创建节点句柄
      ros::NodeHandle n;

      // 创建一个发布者，用于向 "chatter" 话题发布字符串消息
      ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter", 1000);

      // 创建一个订阅器，用于订阅 "chatter" 话题的消息，并调用回调函数进行处理
      ros::Subscriber sub = n.subscribe("chatter", 1000, chatterCallback);

      // 定义一个循环，发布消息并休眠一段时间
      ros::Rate loop_rate(10);
      int count = 0;
      while (ros::ok())
      {
        std_msgs::String msg;
        std::stringstream ss;
        ss << "hello world " << count;
        msg.data = ss.str();

        ROS_INFO("%s", msg.data.c_str());

        chatter_pub.publish(msg);

        ros::spinOnce();

        loop_rate.sleep();
        ++count;
      }

      return 0;
    }
    ```

3. 确保 `CMakeLists.txt` 文件包含以下内容：

    ```cmake
    cmake_minimum_required(VERSION 2.8.3)
    project(talker_listener_demo)

    ## 找到 catkin 并包含设置好的功能
    find_package(catkin REQUIRED COMPONENTS
      roscpp
      std_msgs
    )

    ## 包含 catkin 工程设置的头文件和库
    catkin_package(
      CATKIN_DEPENDS roscpp std_msgs
    )

    ## 添加包含目录
    include_directories(
      ${catkin_INCLUDE_DIRS}
    )

    ## 添加可执行文件
    add_executable(talker_listener_demo src/talker_listener_demo.cpp)
    ## 链接库
    target_link_libraries(talker_listener_demo ${catkin_LIBRARIES})
    ```

4. 构建 Catkin 工作空间：

    ```bash
    cd ~/catkin_ws
    catkin_make
    ```

现在，您已经成功创建了一个简单的 ROS 节点和订阅器的 demo。要运行它，请执行以下步骤：

1. 在一个终端中启动 ROS Master：

    ```bash
    roscore
    ```

2. 在另一个终端中运行 `talker_listener_demo`：

    ```bash
    source ~/catkin_ws/devel/setup.bash
    rosrun talker_listener_demo talker_listener_demo
    ```

您将会看到一个发布者不断地发布 "hello world" 消息，并且一个订阅器接收并打印这些消息。

# ros1 demo（Python）
好的，下面是将 `talker_listener_demo` 从C++版本转换为Python版本的详细步骤和代码示例：

### 步骤 1：创建ROS包

在终端中执行以下命令：

```bash
catkin_create_pkg talker_listener_demo rospy std_msgs
```

这将在你的Catkin工作空间中创建一个名为 `talker_listener_demo` 的ROS包，并将 `rospy` 和 `std_msgs` 添加为依赖项。

### 步骤 2：创建Python脚本

在 `src` 文件夹下创建两个Python脚本：`talker.py` 和 `listener.py`。

```bash
cd ~/catkin_ws/src/talker_listener_demo/src
touch talker.py
touch listener.py
```

### 步骤 3：编写Python节点代码

编辑 `talker.py` 和 `listener.py` 文件，编写发布者节点和订阅者节点的Python代码。

#### talker.py：

```python
#!/usr/bin/env python

import rospy
from std_msgs.msg import String

def talker():
    rospy.init_node('talker', anonymous=True)
    pub = rospy.Publisher('chatter', String, queue_size=10)
    rate = rospy.Rate(1) # 1 Hz
    while not rospy.is_shutdown():
        hello_str = "hello world %s" % rospy.get_time()
        rospy.loginfo(hello_str)
        pub.publish(hello_str)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
```

#### listener.py：

```python
#!/usr/bin/env python

import rospy
from std_msgs.msg import String

def callback(data):
    rospy.loginfo("I heard %s", data.data)

def listener():
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber('chatter', String, callback)
    rospy.spin()

if __name__ == '__main__':
    listener()
```

### 步骤 4：添加依赖项到package.xml（正常应该不需要修改）

在 `talker_listener_demo` 包的 `package.xml` 文件中添加以下内容：

```xml
<depend>rospy</depend>
<depend>std_msgs</depend>
```

### 步骤 5：更新CMakeLists.txt

在 `talker_listener_demo` 包的 `CMakeLists.txt` 文件中确保 `rospy` 和 `std_msgs` 被正确声明为依赖项。

```cmake
find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS rospy std_msgs
)
```

### 编译和运行

在Catkin工作空间中执行以下命令来编译ROS包：

```bash
cd ~/catkin_ws
catkin_make
```
也可以编译指定的

```
catkin_make --only-pkg-with-deps talker_listener_demo
```

然后，在两个不同的终端中分别运行发布者节点和订阅者节点：
执行前需要source一下
```bash
source ~/catkin_ws/devel/setup.bash
```
同时给python脚本添加执行权限

```bash
chmod +x talker.py
chmod +x listener.py
```

可以用下面命令查看包是否存在

```bash
rospack list|grep talker_listener_demo
```
有时python命令不行，需要将python命令指向python3做符号链接

```bash
sudo ln -s /usr/bin/python3 /usr/bin/python
```

```bash
# 首先需要启动Master
roscore

# Terminal 1: Run the talker node
rosrun talker_listener_demo talker.py

# Terminal 2: Run the listener node
rosrun talker_listener_demo listener.py
```

现在，你应该能够在终端中看到发布者节点发布的消息被订阅者节点成功接收并打印出来了。
