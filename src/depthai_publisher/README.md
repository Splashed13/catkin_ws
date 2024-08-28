# depthai_publisher
EGB349 OAK-D Lite DepthAI Publisher

## Ensure you have cv-bridge and vision-opencv packages installed using aptitude store and install depthai packages using pip

```
sudo apt-get install ros-noetic-compressed-image-transport
sudo apt-get install ros-noetic-camera-info-manager
sudo apt-get install ros-noetic-rqt-image-view
sudo apt-get install ros-noetic-cv-bridge
sudo apt-get install ros-noetic-vision-opencv

sudo apt-get install python3-pip
python3 -m pip install -U depthai
```

## Step 1 - Clone Repository

```
cd [ros_ws]/src/
git clone https://github.com/Piaktipik/depthai_publisher.git
```

## Step 2 - Build Packages
```
cd ../
```

#### Source ROS Noetic packages

```
catkin_make
```

## Step 3 - Run the following ros nodes

#### Terminal 1:
```
roscore
```

#### Terminal 2 (run oak-d frame publisher):
```
rosrun depthai_publisher dai_publisher
```

#### Terminal 3 (run arucomarker identification):
```
rosrun depthai_publisher aruco_subscriber
```

#### Terminal 4 (visualisation):
```
rqt_image_view
```
OR
```
rviz
```
