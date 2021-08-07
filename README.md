# Install

文件位于seafile Groups/MPL/MPL/install

## ZSH && OH_MY_ZSH

```sh
sudo apt update
sudo apt-get install -y zsh curl git

sh -c "$(curl -fsSL https://raw.github.com/ohmyzsh/ohmyzsh/master/tools/install.sh)"

```



## ROS

清华源

```sh
sudo sh -c '. /etc/lsb-release && echo "deb http://mirrors.tuna.tsinghua.edu.cn/ros/ubuntu/ $DISTRIB_CODENAME main" > /etc/apt/sources.list.d/ros-latest.list'

sudo apt-key adv --keyserver keyserver.ubuntu.com --recv-keys F42ED6FBAB17C654
```

对于melodic/noetic请分别执行以下命令

```sh
sudo apt update

sudo apt-get install ros-melodic-desktop-full
echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc
echo "source /opt/ros/melodic/setup.zsh" >> ~/.zshrc

source ~/.zshrc

sudo apt install python-rosdep python-rosinstall python-rosinstall-generator python-wstool build-essential
sudo apt install python-rosdep

```

```sh
sudo apt update

sudo apt-get install ros-noetic-desktop-full
echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
echo "source /opt/ros/noetic/setup.zsh" >> ~/.zshrc

source ~/.zshrc

sudo apt install python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential
sudo apt install python3-rosdep
```

最后更新依赖
```sh
sudo rosdep init
rosdep update
```

最后两步出现网络问题则 通过 https://githubusercontent.com.ipaddress.com/raw.githubusercontent.com

找到 raw.githubusercontent.com 的ip 并修改 /etc/hosts  避开dns污染

## 输入法

找到文件 sogoupinyin_xxx_amd64.deb (sogoupinyin_2.4.0.3469_amd64.deb)

```sh
sudo apt-get -y install fcitx
sudo dpkg -i sogoupinyin_2.4.0.3469_amd64.deb
sudo apt -f install
```
打开语言支持(Language Support) 完成更新 并选择fcitx为默认输入法架构

重启
点击屏幕右上角键盘选择config input method
按 + 找到sougoupinyin
完成配置


## VPN

VPN 下载
https://storage.monocloud.co/client/Linux/Clash/Clashy-0.2.0.AppImage



## 换源及相关依赖

切换到清华源（上交，中科大源经常崩勿用） ——通过 Software&Update 软件切换省时省心

```sh
    sudo su

    apt-get update && \
    apt-get upgrade -y && \
    apt-get install -y cmake iputils-ping tftp lsb-core wget && \
    apt-get install -y libeigen3-dev && \
    apt-get install -y libopencv-dev && \
    apt-get install -y libgoogle-glog-dev libgflags-dev && \
    apt-get install -y libatlas-base-dev && \
    apt-get install -y libsuitesparse-dev && \
    apt-get install -y zsh && \
    apt-get install -y libpcl-dev && \
    apt-get install -y libceres-dev && \
    apt-get install -y libprotobuf-dev && \
    apt-get clean

    ln -s /usr/include/eigen3/Eigen /usr/include/Eigen

    apt-get install -y software-properties-common && \
    add-apt-repository -y ppa:borglab/gtsam-release-4.0 && \
    apt update && \
    apt -y install libgtsam-dev libgtsam-unstable-dev

    apt-get install -y openssh-server && \
    apt-get install -y tmux && \
    apt-get install -y sudo vim && \
    apt-get install -y htop
```

重开一个终端
```sh
    sudo apt-get install -y ros-$ROS_DISTRO-pcl-conversions && \
    sudo apt-get install -y ros-$ROS_DISTRO-tf\* && \
    sudo apt-get install -y ros-$ROS_DISTRO-image-\* && \
    sudo apt-get install -y ros-$ROS_DISTRO-wfov-camera-msgs
    
    echo "set -g mouse on" >> ~/.tmux.conf
```


一段直接一起复制，其中安装了 eigen pcl opencv ceres gtsam ros-tf 等常用库 不要忘了开始的 sudo su



## Ros空间建立

```sh
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src

catkin_init_workspace

cd ~/catkin_ws
catkin_make

echo "source ~/catkin_ws/devel/setup.zsh" >> ~/.zshrc
echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
source ~/.zshrc
```

建立一个工作空间即可



## 传感器驱动及rosdriver

### Prophesee

#### 板卡驱动

```sh
unzip SilkyEvCam_G31_Installer_for_ubuntu_v2.5.4.zip
cd SilkyEvCam_G31_Installer_for_ubuntu_v2.5.4

chmod +x CA_Silky_installer.sh
sudo ./CA_Silky_installer.sh
```

zip在文件夹内有提供

注意由于我们是zsh 需要更添加 CA_Silky_installer.sh 中的 bash 为 zsh

```sh
echo "export MV_HAL_PLUGIN_PATH=$MV_HAL_PLUGIN_PATH":$LIB_INSTALL_PATH >> ~/.zshrc
```

#### Metvision
18.04
```sh
sudo su

echo "deb [arch=amd64 trusted=yes] https://prophesee:DbnLdKL5YXnMndWg@apt.prophesee.ai/dists/public/cp51Vn3b/ubuntu bionic essentials" > /etc/apt/sources.list.d/essentials.list && \
    apt update && \
    apt -y install libcanberra-gtk-module mesa-utils && \
    apt -y install 'metavision-*' && \
    apt -y install libboost-program-options-dev
```

20.04
```sh
sudo su

echo "deb [arch=amd64 trusted=yes] https://prophesee:DbnLdKL5YXnMndWg@apt.prophesee.ai/dists/public/cp51Vn3b/ubuntu focal essentials" > /etc/apt/sources.list.d/essentials.list && \
    apt update && \
    apt -y install libcanberra-gtk-module mesa-utils && \
    apt -y install 'metavision-*' && \
    apt -y install libboost-program-options-dev
 ```

之后重启 插上camera 运行 metvision_player 测试

#### Ros driver

首先是openeb

```sh
sudo apt update
sudo apt -y install apt-utils build-essential software-properties-common wget unzip curl
sudo apt -y install cmake  libopencv-dev git # CMake, OpenCV, and Git
sudo apt -y install libboost-all-dev libusb-1.0-0-dev libeigen3-dev # Boost, Libusb and eigen3
sudo apt -y install libglew-dev libglfw3-dev
sudo apt -y install libgtest-dev  # gtest
```

```sh
mkdir ~/libs && cd ~/libs
git clone https://github.com/prophesee-ai/openeb.git
mkdir openeb/build && cd openeb/build
cmake .. -DBUILD_TESTING=OFF -DCOMPILE_PYTHON3_BINDINGS=OFF
cmake --build . --config Release -- -j 4
source ~/libs/openeb/build/utils/scripts/setup_env.sh
sudo cmake --build . --target install
echo "export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/usr/local/lib" >> ~/.zshrc
```

然后是 rosdriver

```sh
cd ~/catkin_ws/src
git clone https://github.com/prophesee-ai/prophesee_ros_wrapper.git
cd ..

catkin_make
```

然后是ros双目driver

```sh
cd ~/catkin_ws/src
git clone https://github.com/prophesee-ai/prophesee_ros_stereo_driver.git
cd ..

catkin_make
```

可以运行 roslaunch prophesee_ros_driver prophesee_publisher.launch 进行测试



### Xsens IMU

#### ros预编译版本

```sh
sudo apt-get install ros-melodic-xsens-driver
sudo chmod 0666 /dev/ttyUSB0
```

可以通过 roslaunch xsens_driver xsens_driver.launch 检查是否成功安装

#### MTmanager使用及ros源码版本安装

解压MT_Software_Suite_linux-x64_2021.0.tar.gz

```sh
sudo apt-get install sharutils
sudo ./mtsdk_linux-x64_2021.0.sh
sudo ./mfmsdk_linux-x64_2021.0.sh
sudo ldconfig /usr/local/xsens

```

```sh
sudo cp -r /usr/local/xsens/xsens_ros_mti_driver ~/catkin_ws/src/
cd ~/catkin_ws
catkin_make
```

若出现问题则 尝试 https://answers.ros.org/question/358786/xsense-ros-kinetic-package-error-usrbinld-cannot-find/ 解决

解压 mtmanager_linux-x64_2021.0.tar.gz

```sh
sudo apt-get install qtcreator
sudo apt-get install qt5-default
sudo apt-get install libxcb*

cp -r mtmanager ~/libs
sudo ln -s ~/libs/mtmanager/linux-x64/bin/mtmanager /usr/bin/
```

尝试运行 mtmanager 成功打开UI 界面则成功



### ouster

首先网线连接好之后将主机设置为静态ip 192.168.1.100  mask:255.255.255.0 雷达地址为192.168.1.2

#### 安装SDK

```sh
sudo pip3 install --upgrade pip
python3 -m pip install ouster-sdk
```

#### 安装驱动及ros功能包

```sh
sudo apt install -y build-essential cmake libglfw3-dev libglew-dev libeigen3-dev libjsoncpp-dev libtclap-dev
cd ~/libs
git clone https://github.com/ouster-lidar/ouster_example
cd ouster_example 
mkdir build && cd build
cmake -DCMAKE_BUILD_TYPE=Release ..
make -j 4
```

```sh
sudo apt install -y ros-$ROS_DISTRO-ros-core ros-$ROS_DISTRO-pcl-ros ros-$ROS_DISTRO-tf2-geometry-msgs ros-$ROS_DISTRO-rviz
rosdep install --from-paths ~/libs/ouster_example
ln -s ~/libs/ouster_example ~/catkin_ws/src/
catkin_make -DCMAKE_BUILD_TYPE=Release
```

若要运行ouster 则使用

```
roslaunch ouster_ros ouster.launch sensor_hostname:=192.168.1.2 udp_dest:=192.168.1.100 metadata:=$HOME/Documents/Ouster/temp.json lidar_mode:=2048x10 viz:=false
```

若需要rviz可视化则添加viz:=true



### Kinect Azure

#### k4a-tools

https://docs.microsoft.com/zh-cn/azure/Kinect-dk/sensor-sdk-download

```sh
curl -sSL https://packages.microsoft.com/keys/microsoft.asc | sudo apt-key add -
sudo apt-add-repository https://packages.microsoft.com/ubuntu/18.04/prod
sudo apt-get update
sudo apt-get install k4a-tools
```

#### libk4a预编译版本

```
sudo apt install libk4a1.4-dev
```

#### libk4a源码版本

```
sudo dpkg --add-architecture amd64
sudo apt update
sudo apt install -y \
    pkg-config \
    ninja-build \
    doxygen \
    clang \
    gcc-multilib \
    g++-multilib \
    python3 \
    nasm \
    cmake \
    libgl1-mesa-dev \
    libsoundio-dev \
    libvulkan-dev \
    libx11-dev \
    libxcursor-dev \
    libxinerama-dev \
    libxrandr-dev \
    libusb-1.0-0-dev \
    libssl-dev \
    libudev-dev \
    mesa-common-dev \
    uuid-dev
```

```sh
cd ~/libs
git clone -b v1.4.1 https://github.com/microsoft/Azure-Kinect-Sensor-SDK.git
cd Azure-Kinect-Sensor-SDK
sudo apt-get install ninja-build libsoundio-dev
mkdir build && cd build
cmake .. -GNinja
ninja -j 8
sudo ninja install
```

详见 https://github.com/microsoft/Azure-Kinect-Sensor-SDK

https://zhuanlan.zhihu.com/p/153125845 可能有深度库需要链接，如果图省事就直接预编译版本

#### RosDriver

```sh
cd ~/catkin_ws/src
git clone https://github.com/microsoft/Azure_Kinect_ROS_Driver.git
cd ~/catkin_ws
catkin_make
```



### PointGray

#### 安装SDK
解压spinnaker-2.4.0.143-Ubuntu18.04-amd64-pkg.tar.gz
阅读readme 安装prerequisite

```sh
sudo apt-get install libavcodec57 libavformat57 libswscale4 libswresample2 libavutil55 libusb-1.0-0
```

安装SPINVIEWER

````sh
sudo sh install_spinnaker.sh
````

*其中有一条是添加用户到用户组，输入本用户的名字 注意！！

修改USB缓存, 由于有两个相机并且还有KINECT + PROPHESEE 直接调整为2560m 较为保险

````sh
sudo sh configure_usbfs.sh
````

测试使用 

````
spinview
````

#### ROS Driver

```sh
cd ~/catkin_ws/src
git clone https://github.com/ros-drivers/flir_camera_driver
cd ~/catkin_ws
catkin_make
```

