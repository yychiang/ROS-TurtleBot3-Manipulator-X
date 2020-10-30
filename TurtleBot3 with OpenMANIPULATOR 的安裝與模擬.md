###### tags: `ROS`

# TurtleBot3 with OpenMANIPULATOR 的安裝與模擬

## 1. 先安裝`Ubuntu 16.04`

[**Ubuntu下載的連結**](https://www.ubuntu.com/download/alternative-downloads)

一步一步的安裝導引：[**Install Ubuntu Desktop**](https://www.ubuntu.com/download/desktop/install-ubuntu-desktop)
## 2. 安裝ROS系統
### 2.1 Setup your sources.list
Setup your computer to accept software from packages.ros.org.

```
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
```
### 2.2 Set up your keys
```bash
sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
```

### 2.3 Installation


First, make sure your Debian package index is up-to-date:
```bash
sudo apt-get update
```

**Desktop-Full Install: (Recommended):**
```
sudo apt-get install ros-kinetic-desktop-full
```

### 2.4 Environment setup

```bash
echo "source /opt/ros/kinetic/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

### 2.5 Dependencies for building packages
```bash
sudo apt install python-rosdep python-rosinstall python-rosinstall-generator python-wstool build-essential
```

#### 2.5.1 Initialize rosdep
Before you can use many ROS tools, you will need to initialize rosdep. rosdep enables you to easily install system dependencies for source you want to compile and is required to run some core components in ROS. If you have not yet installed rosdep, do so as follows.
```bash
sudo apt install python-rosdep
```
With the following, you can initialize rosdep.
```bash
sudo rosdep init
rosdep update
```

### 2.6 測試
#### 2.6.1 測試`roscore`及`Gazebo`
在終端機鍵入：
```bash
$ roscore
```
看到下列結果表示成功。
![](https://i.imgur.com/Vlsfy0d.png)

接下來測試`Gazebo`。在終端機鍵入：
```
$ gazebo
```

如果是用VMWare的同學，會出現錯誤訊息或閃退：
```
VMware: vmw_ioctl_command error Invalid argument.
```

必須下此命令修改：
```
$ echo "export SVGA_VGPU10=0" >> ~/.bashrc
```

如果能成功把Gazebo打開，算是成功了。

#### 2.6.2 測試`TurtleSim`


## 3. 建立Catkin工作空間
### 第1步：建立catkin工作空間及其子目錄
在本系列課程所開發所有程式碼，都位於catkin工作空間中。我們只需要建立並初始化該工作空間一次即可。
首先，建立最上層資料匣，常命名為`catkin_ws`資料匣，及其中的一個子目錄：`src`(唸作source)。
`mkdir -p ~/catkin_ws/src`

### 第2步：造訪到source資料匣
`cd ~/catkin_ws/src`

### 第3步：初始化catkin工作空間
初始化工作空間之後，會產生一個`CMakeLists.txt`的檔案。初始化請鍵入：
`$ catkin_init_workspace`

![](https://i.imgur.com/e704NKV.png)

您可試試現在這個目錄新增了甚麼：
`$ ls -l`
請注意這個符號連結(symbolic link)`CMakeLists.txt`已經被建立在
`/opt/ros/kinetic/share/catkin/cmake/toplevel.cmake`

### 第4步：回到最上層目錄
回到`catkin_ws`：
`$ cd ~/catkin_ws`

### 第５步：構建(build)工作空間
`$ catkin_make`

這時會看到：
```
-- BUILD_SHARED_LIBS is on
-- Configuring done
-- Generating done
-- Build files have been written to: /home/workspace/catkin_ws/build
####
#### Running command: "make -j4 -l4" in "/home/workspace/catkin_ws/build"
####
```
我們可看看又新增了甚麼：
`$ ls`

![](https://i.imgur.com/W1P86Wd.png)

會看到兩個新增的資料匣：
* `build`：這是`C++`程式包的構建空間，大多數時候不用去管它。
* `devel`：內部會有一個`setup.bash`的檔案。開始使用catkin工作空間之前，必須先source這個檔案：
`$ source devel/setup.bash`

至此，便已將catkin工作空間建立完成。

最後，修改一下`~/.bashrc`：
```
gedit ~/.bashrc
```
最後要有這兩行：
```
source /opt/ros/kinetic/setup.bash
source ~/catkin_ws/devel/setup.bash
```

如下圖：
![](https://i.imgur.com/SiltD8h.png)

## 4. Install Dependent ROS 1 Packages
The next step is to install dependent packages for TurtleBot3 control on Remote PC.
```
$ sudo apt-get install ros-kinetic-joy ros-kinetic-teleop-twist-joy ros-kinetic-teleop-twist-keyboard ros-kinetic-laser-proc ros-kinetic-rgbd-launch ros-kinetic-depthimage-to-laserscan ros-kinetic-rosserial-arduino ros-kinetic-rosserial-python ros-kinetic-rosserial-server ros-kinetic-rosserial-client ros-kinetic-rosserial-msgs ros-kinetic-amcl ros-kinetic-map-server ros-kinetic-move-base ros-kinetic-urdf ros-kinetic-xacro ros-kinetic-compressed-image-transport ros-kinetic-rqt-image-view ros-kinetic-gmapping ros-kinetic-navigation ros-kinetic-interactive-markers
```

```
$ cd ~/catkin_ws/src/
$ git clone https://github.com/ROBOTIS-GIT/turtlebot3_msgs.git
$ git clone -b kinetic-devel https://github.com/ROBOTIS-GIT/turtlebot3.git
$ cd ~/catkin_ws && catkin_make
```
If `catkin_make` command is completed without any errors, the preparation for TurtleBot3 is done.

切換到`src`看一下安裝了甚麼：
![](https://i.imgur.com/FaSOhMh.png)

## 5. Simulation
先安裝：
```bash
$ cd ~/catkin_ws/src/
$ git clone https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git
$ cd ~/catkin_ws && catkin_make
```
### 5.1 [TurtleBot3 Simulation using Gazebo](https://emanual.robotis.com/docs/en/platform/turtlebot3/simulation/#turtlebot3-simulation-using-gazebo)

#### 5.1.1 [**Simulate in Various World**](https://emanual.robotis.com/docs/en/platform/turtlebot3/simulation/#simulate-in-various-world)

**(1) Empty World**

依據型號，以下二選一：
```bash
$ export TURTLEBOT3_MODEL=burger
$ roslaunch turtlebot3_gazebo turtlebot3_empty_world.launch
```
或
```bash
$ export TURTLEBOT3_MODEL=waffle_pi
$ roslaunch turtlebot3_gazebo turtlebot3_empty_world.launch
```

可看到如下畫面：
![](https://i.imgur.com/C4tvgha.png)
或
![](https://i.imgur.com/uoxjPC9.png)

**(2) TurtleBot3 World**

```bash
$ export TURTLEBOT3_MODEL=waffle_pi
$ roslaunch turtlebot3_gazebo turtlebot3_world.launch
```
![](https://i.imgur.com/pl6d7F3.png)

![](https://i.imgur.com/PPtoexz.png)

![](https://i.imgur.com/sJejlXb.png)

**(3) TurtleBot3 House**

```bash
$ export TURTLEBOT3_MODEL=waffle_pi
$ roslaunch turtlebot3_gazebo turtlebot3_house.launch
```

![](https://i.imgur.com/mQrr6kM.png)

![](https://i.imgur.com/Cl6sCil.png)

#### 5.1.2 [**Drive TurtleBot3**](https://emanual.robotis.com/docs/en/platform/turtlebot3/simulation/#drive-turtlebot3)

這一小節，可做到任選一隻機器人在上述三種環境中避障(當然，選擇空的環境是沒有意義的)。

**(1) Teleoperation on Gazebo (鍵盤操控)**

```bash
$ export TURTLEBOT3_MODEL=burger
$ roslaunch turtlebot3_gazebo turtlebot3_world.launch
```

Open a new terminal window and enter below command.
```bash
$ export TURTLEBOT3_MODEL=burger
$ roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch
```

**(2) Collision Avoidance (避障)**

```bash
$ export TURTLEBOT3_MODEL=waffle_pi
$ roslaunch turtlebot3_gazebo turtlebot3_world.launch
```
Open a new terminal window and enter below command.
```bash
$ export TURTLEBOT3_MODEL=waffle_pi
$ roslaunch turtlebot3_gazebo turtlebot3_simulation.launch
```
[TB3避障的影片](https://youtu.be/qvEQ4U32keQ)

#### 5.1.3 [**Execute RViz**](https://emanual.robotis.com/docs/en/platform/turtlebot3/simulation/#execute-rviz)

(待續)

## 6. 安裝OpenManipulator-x
### 6.1 安裝
更新：
```bash
$ sudo apt-get update && sudo apt-get upgrade
```


Binary安裝:
```bash
$ sudo apt-get install ros-kinetic-ros-controllers ros-kinetic-gazebo* ros-kinetic-moveit* ros-kinetic-industrial-core
```
Git安裝:
```bash
$ cd ~/catkin_ws/src/
$ git clone https://github.com/ROBOTIS-GIT/DynamixelSDK.git
$ git clone https://github.com/ROBOTIS-GIT/dynamixel-workbench.git
$ git clone https://github.com/ROBOTIS-GIT/dynamixel-workbench-msgs.git
$ git clone https://github.com/ROBOTIS-GIT/open_manipulator.git
$ git clone https://github.com/ROBOTIS-GIT/open_manipulator_msgs.git
$ git clone https://github.com/ROBOTIS-GIT/open_manipulator_simulations.git
$ git clone https://github.com/ROBOTIS-GIT/robotis_manipulator.git
$ cd ~/catkin_ws && catkin_make
```

成功了：
![](https://i.imgur.com/r30bhPl.png)

看看多了甚麼?
![](https://i.imgur.com/JvRd3Wa.png)


### 6.2 [[ROS] Controller Package](https://emanual.robotis.com/docs/en/platform/openmanipulator_x/ros_controller_package/#ros-controller-package)

You can control each joint of OpenMANIPULATOR-X and check states of OpenMANIPULATOR-X through [messages](https://emanual.robotis.com/docs/en/platform/openmanipulator_x/ros_controller_package/#message-list) by utilizing an exclusive controller program.

Before launching the controller, please check `open_manipulator_controller` launch file in `open_manipulator_controller` package.

```bash
$ cd ~/catkin_ws/src/open_manipulator/open_manipulator_controller/launch
$ gedit open_manipulator_controller.launch
```

修改為：
```xml=
<launch>
    <arg name="use_robot_name"         default="open_manipulator"/>

    <arg name="dynamixel_usb_port"     default="/dev/ttyUSB0"/>
    <arg name="dynamixel_baud_rate"    default="1000000"/>

    <arg name="control_period"         default="0.010"/>

    <arg name="use_platform"           default="true"/>

    <arg name="use_moveit"             default="false"/>
    <arg name="planning_group_name"    default="arm"/>
    <arg name="moveit_sample_duration" default="0.050"/>

    <group if="$(arg use_moveit)">
      <include file="$(find open_manipulator_controller)/launch/open_manipulator_moveit.launch">
        <arg name="robot_name"      value="$(arg use_robot_name)"/>
        <arg name="sample_duration" value="$(arg moveit_sample_duration)"/>
      </include>
    </group>

    <node name="$(arg use_robot_name)" pkg="open_manipulator_controller" type="open_manipulator_controller" output="screen" args="$(arg dynamixel_usb_port) $(arg dynamixel_baud_rate)">
        <param name="using_platform"       value="$(arg use_platform)"/>
        <param name="using_moveit"         value="$(arg use_moveit)"/>
        <param name="planning_group_name"  value="$(arg planning_group_name)"/>
        <param name="control_period"       value="$(arg control_period)"/>
        <param name="moveit_sample_duration"  value="$(arg moveit_sample_duration)"/>
    </node>

  </launch>  
  ```
  
  ### 6.3 模擬
  以下三小節的命令，開三個終端機視窗執行之後，就可以控制機械手臂了!
  #### 6.3.1 [Launch gazebo](https://emanual.robotis.com/docs/en/platform/openmanipulator_x/ros_simulation/#launch-gazebo)

```bash
 $ roslaunch open_manipulator_gazebo open_manipulator_gazebo.launch
```
  
  #### 6.3.2 Open [OpenManipulator control GUI](https://emanual.robotis.com/docs/en/platform/openmanipulator_x/ros_operation/#gui-program)

```bash
$ roslaunch open_manipulator_control_gui open_manipulator_control_gui.launch
```
  
  #### 6.3.3 [Controller for gazebo](https://emanual.robotis.com/docs/en/platform/openmanipulator_x/ros_simulation/#controller-for-gazebo)
  
```bash
 $ roslaunch open_manipulator_controller open_manipulator_controller.launch use_platform:=false
```  
  
![](https://i.imgur.com/laAUo74.png)
  