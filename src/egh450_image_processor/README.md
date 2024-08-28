# egh450_image_processor
An example package to subscribe to ROS images and process them using OpenCV

## Download & Compile
```sh
cd ~/catkin_ws/src
git clone https://github.com/qutas/egh450_image_processor
cd ~/catkin_ws
catkin_make
source ~/catkin_ws/devel/setup.bash
```

## Usage
To run a basic demonstration off of a sample video, first place your video into the launch folder, and rename it to: `test_vid.mp4`. Then run the command:
```sh
roslaunch egh450_image_processor processor_demo.launch
```

Notes:
- Be aware that once the video duration is completed, the last frame will be repeatedly transmitted. To start the video over, simply close and re-open the launch file
- Open another terminal and run `rqt` to visualize your video stream.
