# 1. 更新软件包列表
```
sudo apt update
```

# 2. 升级所有软件包
```
sudo apt upgrade
```

# 3. 解决右上角红点（如果上一条命令执行后红点未消失）
```
sudo apt-get install ubuntu-pro-client:amd64
```

# 4. 安装基础软件
```
sudo apt install openssh-server
sudo apt install curl
sudo apt install git
sudo apt install net-tools
```

# 5. 安装todesk
```
sudo apt-get install ./todesk-v4.7.2.0-amd64.deb
```

# 6. 安装qt
```
chmod +x qt-opensource-linux-x64-5.14.2.run
```

```
// 以下在图形界面执行
sudo ./qt-opensource-linux-x64-5.14.2.run
```

# 7. 安装docker
```
// 安装依赖包
sudo apt install apt-transport-https ca-certificates curl software-properties-common
```

```
// 添加 Docker 的官方 GPG 密钥
curl -fsSL https://download.docker.com/linux/ubuntu/gpg | sudo apt-key add -
```

```
// 添加 Docker 软件源
sudo add-apt-repository "deb [arch=amd64] https://download.docker.com/linux/ubuntu $(lsb_release -cs) stable"
```

```
// 更新软件包列表
sudo apt update
```

```
// 安装 Docker CE（Community Edition）
sudo apt install docker-ce
```

```
// 启动并启用 Docker 服务
sudo systemctl start docker
sudo systemctl enable docker
```

```
// 检查 Docker 是否安装成功
docker --version
```

# 8. 安装docker-compose
```
// 下载 Docker Compose 二进制文件
sudo curl -L "https://github.com/docker/compose/releases/download/v2.20.2/docker-compose-$(uname -s)-$(uname -m)" -o /usr/local/bin/docker-compose
```

```
// 给文件赋予可执行权限
sudo chmod +x /usr/local/bin/docker-compose
```

```
// 测试 Docker Compose 是否安装成功
docker-compose --version
```

# 9. 安装编译套件和图像相关库
```
sudo apt install build-essential
sudo apt install cmake g++ libfmt-dev
sudo apt install libgl1-mesa-dev
sudo apt install libglu1-mesa-dev libglew-dev
```

# 10. 非root目录赋权
```
sudo chown -R cddz:cddz /work/ch
```

# 11. Ubuntu上安装ROS Noetic

### 1. **设置Ubuntu的安装源**

首先确保系统是Ubuntu 20.04，因为ROS Noetic仅支持该版本。然后，配置系统使其可以从ROS的软件仓库中获取安装包。

#### Step 1: 更新软件源
```bash
sudo apt update
sudo apt upgrade
```

#### Step 2: 添加ROS Noetic软件源

```bash
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-noetic.list'
```

#### Step 3: 添加ROS的GPG密钥

```bash
curl -sSL http://packages.ros.org/ros.key | sudo apt-key add -
```

### 2. **安装ROS Noetic**

#### Step 1: 更新APT索引

```bash
sudo apt update
```

#### Step 2: 安装ROS Noetic

有三种安装方式：

- **桌面版（推荐）**：安装常用的ROS工具、Rviz、Gazebo等。
  ```bash
  sudo apt install ros-noetic-desktop-full
  ```

- **桌面轻量版**：安装常用的ROS工具和Rviz，不包括模拟器等。
  ```bash
  sudo apt install ros-noetic-desktop
  ```

- **基础版**：只安装ROS的基础库，不包括GUI工具。
  ```bash
  sudo apt install ros-noetic-ros-base
  ```

### 3. **初始化rosdep**

`rosdep`是ROS的一个依赖管理工具。首次使用前需要初始化：

```bash
sudo apt install python3-rosdep
sudo rosdep init
rosdep update
```

### 4. **设置ROS环境变量**

每次启动新的终端时，都需要设置ROS环境。你可以通过以下命令手动设置，也可以把它加入到`.bashrc`文件中：

#### 手动设置：
```bash
source /opt/ros/noetic/setup.bash
```

#### 自动设置（推荐）：
```bash
echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

### 5. **安装依赖工具**

如果要编译ROS包，建议安装以下工具：

```bash
sudo apt install python3-rosinstall python3-rosinstall-generator python3-wstool build-essential
```

### 6. **创建工作空间（可选）**

如果你打算开发自己的ROS包，可以创建一个catkin工作空间：

```bash
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/
catkin_make
source devel/setup.bash
```

### 7. **验证安装**

可以通过以下命令启动ROS核心节点，来验证ROS是否安装成功：

```bash
roscore
```

同时可以启动Rviz查看界面工具：

```bash
rviz
```

这就完成了ROS Noetic的安装。

# 12、运行镜像相关
```
sudo docker build -t foxy_noetic_ubuntu20.04:v1 .
chmod +x start.sh
sudo docker exec -it dadp-ros-dev /bin/bash
```
