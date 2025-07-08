# 2025国机赛无人机代码

## nuc环境配置

参考notion中哨兵系统重装指南 https://www.notion.so/16c24a7052ae80909490d654ea911af2

### 1.安装梯子

推荐一个Releases · clash-verge-rev/clash-verge-rev (github.com)

配置终端代理：在梯子内查看端口号

```bash
export https_proxy=http://127.0.0.1:<端口>
export http_proxy=http://127.0.0.1:<端口>
export all_proxy=socks5://127.0.0.1:<端口>
```

### 2.换源&安装ROS2

```bash
wget http://fishros.com/install -O fishros && . fishros
```

### 3.配置本地ssh

下载并配置git

```bash
sudo apt install git
git config --global user.name "Your Name"
git config --global user.email "your_email@example.com"
git config --global http.proxy http://127.0.0.1:<端口> #<步骤1处的代理服务器地址>
git config --global https.proxy https://127.0.0.1:<端口> #<步骤1处的代理服务器地址>
```

可通过查看代理服务器地址与端口

```bash
echo $http_proxy
echo $https_proxy
```

生成新的SSH密钥，一路Enter即可

```bash
ssh-keygen -t rsa -C "github注册@邮箱.com"
```

复制下面得到的密钥至githubSSHkeys

```bash
cd ~/.ssh
cat id_rsa.pub
```

换ssh端口

```bash
gedit ~/.ssh/config
```

添加

```bash
Host github.com
  HostName ssh.github.com
  Port 443
  User git
```

### 4.安装vscode并登录

### 5.开启性能模式，设置节电选项，设置电源按键行为，开启自动登录

### 6.关闭自动休眠

参考：
https://blog.csdn.net/qq_38429958/article/details/131763351?sharetype=blog&shareId=131763351&sharerefer=APP&sharesource=2301_79971666&sharefrom=link

```bash
sudo systemctl mask sleep.target suspend.target
#查看状态休眠模式确认，sleep状态为masked,表示自动休眠模式已关闭。
systemctl status sleep.target
```

### 7.开虚拟内存（一般开16GB）

来自参考：https://blog.csdn.net/WU2629409421perfect/article/details/118878100

```bash
# 查看当前虚拟内存大小
free -m 

#新建目录存放swapfile
sudo mkdir /swap
cd /swap
#生成swapfile文件
sudo dd if=/dev/zero of=swapfile bs=1024 count=7000000
#将生成的文件转换成swap文件
sudo mkswap swapfile
#激活swap文件
sudo swapon swapfile

#修改为随系统启动自动挂载生效
sudo gedit /etc/fstab

#在最后追加一行!!!
/swap/swapfile swap swap defaults 0 0
```

### 8.设置按键一下立刻关机（改完了之后按开机键可以关机）

参考：https://blog.csdn.net/weixin_44517278/article/details/131008835

```bash
sudo touch /etc/acpi/events/powerbtn
sudo gedit /etc/acpi/events/powerbtn
```

在其中添加以下内容：

```bash
event=button/power
action=/sbin/poweroff
```

重启acpid服务，让修改生效：

```bash
sudo systemctl restart acpid
```

### 9.卸载brltty

```bash
sudo apt remove brltty
```

### 10.给串口权限

参考：https://blog.csdn.net/Android_WPF/article/details/120892617?ops_request_misc=%257B%2522request%255Fid%2522%253A%2522170914546316800186534822%2522%252C%2522scm%2522%253A%252220140713.130102334..%2522%257D&request_id=170914546316800186534822&biz_id=0&utm_medium=distribute.pc_search_result.none-task-blog-2~all~sobaiduend~default-2-120892617-null-null.142^v99^control&utm_term=ubuntu串口权限&spm=1018.2226.3001.4187

```bash
sudo gedit /etc/udev/rules.d/70-ttyusb.rules
```

在该文件中添加如下一行（可能不存在此文件而创建一个新文件）

```bash
KERNEL==“ttyUSB[0-9]*”, MODE=“0666”
```

### 11.USB口重命名为drone_serial

来自参考：https://blog.csdn.net/xx970829/article/details/115529204

查看要改名的USB的ID

```bash
lsusb #拔插usb确定是哪一个
cd etc/udev/rules.d
sudo touch sentry.rules
sudo gedit sentry.rules
```

在里面输入： 其中的“067b”，"2303"为usb线的ID

```bash
KERNEL=="ttyUSB*", ATTRS{idVendor}=="067b", ATTRS{idProduct}=="2303", MODE:="0777", SYMLINK+="drone_serial "
```

### 12.安装nav2

```bash
sudo apt install ros-humble-nav2-*
```

### 13.livox驱动

```bash
mkdir -p Livox/src
cd Livox/src
```

参考队内仓库livox_ros_driver2中的readme安装，安装完成后修改下面的路径并添加到.bashrc中

```bash
source "path/to/livox_ws"/install/setup.bash
```

### 14.安装pcl-tools

```bash
sudo apt install libpcl-dev
sudo apt install pcl-tools
```

### 15.安装rqt-tf-tree

```bash
sudo apt install ros-humble-rqt-tf-tree 
#如果安装完了但是rqt不显示，要清除rqt缓存
rm ~/.config/ros.org/rqt_gui.ini
```

### 16.修改有线连接，并连接雷达测试

（参考图片）



附：四元数欧拉角在线转换 
https://www.itutool.com/tools/euler-quaternion-converter

## pymavlink相关安装使用记录

### 下载mavlink

```bash
git clone https://github.com/mavlink/mavlink.git
```

### 安装pymavlink

#### 克隆仓库

```bash
git clone https://github.com/ArduPilot/pymavlink.git
```

#### 安装依赖

```bash
sudo apt-get install libxml2-dev libxslt-dev
sudo apt-get install python3-numpy python3-pytest
sudo python3 -m pip install --upgrade future lxml
```

#### 编译安装（For developers）

先将mavlink包中的message_definitions文件夹复制粘贴到pymavlink文件夹下，并切换进pymavlink文件夹

```bash
sudo MDEF=PATH_TO_message_definitions python3 -m pip install . -v
sudo python3 setup.py install
```

## px4+gazebo仿真环境配置

### 安装PX4固件

下载源码 --recursive会递归下载二级及以下文件

```bash
git clone https://github.com/PX4/PX4-Autopilot.git --recursive
```

进入PX4-Autopilot文件夹，确认源码是否下载完毕，如果因网络等未下载完全，则需要进一步下载更新

```bash
cd PX4-Autopilot
git submodule update --init --recursive
```

源码下载完毕后，执行下面操作

```bash
bash ./Tools/setup/ubuntu.sh
```

显示 relogin or reboot when you attempting to install Nuttx即说明上一步成功

```bash
make px4_sitl gz_x500
```

成功后将会显示Gazebo界面，并且里面有一架飞机

### 安装XRCE-DDS代理

下载源码

```bash
git clone https://github.com/eProsima/Micro-XRCE-DDS-Agent.git
```

编译安装

```bash
cd Micro-XRCE-DDS-Agent
mkdir build
cd build
cmake ..
make
sudo make install
sudo ldconfig /usr/local/lib/
```

安装完成后，启动代理，端口号为8888

```bash
MicroXRCEAgent udp4 -p 8888
```

### 安装QGC地面站

```bash
sudo usermod -a -G dialout $USER
sudo apt-get remove modemmanager -y
sudo apt install gstreamer1.0-plugins-bad gstreamer1.0-libav gstreamer1.0-gl -y
sudo apt install libfuse2 -y
sudo apt install libxcb-xinerama0 libxkbcommon-x11-0 libxcb-cursor-dev -y
```

然后注销用户重新登录

下载软件

```bash
wget https://d176tv9ibo4jno.cloudfront.net/latest/QGroundControl.AppImage
```

安装

```bash
chmod +x ./QGroundControl.AppImage
```

运行

```bash
./QGroundControl.AppImage
```

仿真时需要同时运行地面站，否则无法进行起飞降落等指令

### 建仿真样例的工作空间（可选）

```bash
mkdir -p ~/ws_ros2/src/
cd ~/ws_ros2/src/
git clone https://github.com/PX4/px4_msgs.git
git clone https://github.com/PX4/px4_ros_com.git
```

编译运行

```bash
cd ..
colcon build
source install/local_setup.bash
ros2 run px4_ros_com offboard_control
```