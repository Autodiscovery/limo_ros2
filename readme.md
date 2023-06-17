# What is this repository

This is a fork of https://github.com/agilexrobotics/limo_ros2 that aims to improve on english language support and provide additional resources for education.
We also aim to follow ROS 2 naming conventions and best practices.

# Directly on robot：Running on the Jetson Nano:

## Environment Configuration

Setup the environment for running ROS 2 on LIMO robot

## Local deployment

Use rosdep to install required packages

# Install essential packages

```bash
apt-get update
apt-get install -y --no-install-recommends libusb-1.0-0 udev apt-transport-https ca-certificates curl swig software-properties-common python3-pip
sudo apt install python3-colcon-common-extensions
```

# Install ROS 2

First ensure that the Ubuntu Universe repository is enabled.

```bash
sudo apt install software-properties-common
sudo add-apt-repository universe
```

Now add the ROS 2 GPG key with apt.

```bash
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
```

Then add the repository to your sources list.

```bash
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
```

Install common packages.

```bash
sudo apt update && sudo apt install -y \
  python3-flake8-docstrings \
  python3-pip \
  python3-pytest-cov \
  ros-dev-tools
```

Install Ubuntu version specific packages

```bash
sudo apt-get install python3-vcstool
python3 -m pip install -U \
   flake8-blind-except \
   flake8-builtins \
   flake8-class-newline \
   flake8-comprehensions \
   flake8-deprecated \
   flake8-import-order \
   flake8-quotes \
   "pytest>=5.3" \
   pytest-repeat \
   pytest-rerunfailures
```

```bash
mkdir -p ~/ros2_humble/src
cd ~/ros2_humble
vcs import --input https://raw.githubusercontent.com/ros2/ros2/humble/ros2.repos src
```

### Clone repository
```bash
# Create local workspace
mkdir -p ~/limo_ros2_ws
cd ~/limo_ros2_ws
# Clone the project
git clone https://github.com/Autodiscovery/limo_ros2 src

mv ~/limo_ros2_ws/src/.devcontainer ~/limo_ros2_ws

# Install ydlidar driver
cd ..
git clone https://ghproxy.com/https://github.com/YDLIDAR/YDLidar-SDK.git
mkdir -p YDLidar-SDK/build
cd YDLidar-SDK/build
cmake
make
sudo make install
cd ..
pip install . 
cd ..

# Compile limo_ros2 packages
cd ~/limo_ros2_ws
colcon make
source devel/setup.bash
```

# Use Docker (alternative to direct insallation)

dockerfile is available from docker and can be used instead of direcly installing libraries on the device：

``【推荐】使用 VS Code remote 插件 连接到 limo，打开 ~/agx_workspace 后在菜单中选择 reopen in container``
 ``[Recommend] Login the limo via VS Code remote plugin, open ~/agx_workspace.Then select reopen in container in the menu``

`` Or running automatically setup script``

```shell
cd ~/agx_workspace/src
chmod +x setup.sh
./docker_setup.sh
```
Then follow the prompts

### Get Docker image

Agilex have also packaged and uploaded the required image to the dockerhub website, you can run the following command directly:：

```bash
docker pull lagrangeluo/limo_ros2:v1
```

After the image is pulled, check the image. If the image name appears in the list, the pull is successful.

```bash
agilex@agilex-desktop:~$ docker image list
REPOSITORY                                                         TAG        IMAGE ID       CREATED          SIZE
lagrangeluo/limo_ros2                                              v1         224540b5b168   11 minutes ago   7.57GB
```

Start the image through the container:

```bash
docker run --network=host \
      -d
      -v=/dev:/dev \
      --privileged \
      --device-cgroup-rule="a *:* rmw" \
      --volume=/tmp/.X11-unix:/tmp/.X11-unix \
      -v $XAUTH:$XAUTH -e XAUTHORITY=$XAUTH \
      --runtime nvidia
      --gpus=all \
      -w=/workspace \
      --name limo_dev \
      -e LIBGL_ALWAYS_SOFTWARE="1" \
      -e DISPLAY=${DISPLAY} \
      --restart=always \
      -v ~/agx_workspace:/home/limo_ros1_ws \
      lagrangeluo/limo_ros2:v1 \

```

After the container is successfully created by any method, the code can be run in the container environment.

# Navigation

In your workspace:

```shell
rviz2
## start the chassis
ros2 launch limo_bringup limo_start.launch.py
sleep 2

## start navigation
ros2 launch limo_bringup navigation2.launch.py
```

# start positioning

```shell
rviz2
ros2 launch limo_bringup limo_start.launch.py
ros2 launch build_map_2d revo_build_map_2d.launch.py
#After the above three command are activated use a separate screen to control the car
```

# keyboard control

```shell
ros2 launch limo_bringup limo_start.launch.py
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

# Simulation

## Four wheel differential mode

Display the model in rviz2.

```
ros2 launch limo_description display_models_diff.launch.py 
```

Run the simulation in gazebo

```
ros2 launch limo_description gazebo_models_diff.launch.py 
```

Start keyboard teleop to control Limo

```
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

## Ackermann Model

In development...
