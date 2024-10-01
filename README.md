# EGH450
Github repository for all software developed by Group 5 throughout EGH450

## Using disros to distribute the network
disros is a custom function that is defined in ~/.bashrc and called everytime a new terminal is opened

## QUT Flight Stack
see: https://github.com/qutas/info/wiki/UAV-Setup-Guides-(2024)#the-qutas-flight-stack


## Setup Simulation Environment
1. Open terminal and run

   ```roscore```
2. Open another terminal and navigate to the EGH450 folder `cd ~/EGH450` then run:

   ```roslaunch qutas_lab_450 environment.launch```
3. Open another terminal and run:

   ```roslaunch spar_node spar_uavasr.launch```

4. Breadcrumb to be integrated (usually run here in this step....) will be run with `roslaunch breadcrumb breadcrumb.launch`

**Breadcrumb subscribes**
Subscribed Topics
Occupancy Grid
Topic: grid
Type: nav_msgs/OccupancyGrid
Description: This input sets the grid that is used by breadcrumb to perform the path planning steps. This should be connected to a pre-determined occupancy grid.

**qutas_lab_450 publishes occupancy grid**
There are two modes available for defining obstacles within the flight area when using the `qutas_lab_450/area_map_node` node. The `environment.launch` file automatically loads in the file `launch/map_layout.yaml`. In this file, the obstacles are defined in the `obstacles:` parameters.

When the node first loads, it checks to see if any obstacles have been statically defined. If some are found, these obstacles are used to generate the desired layout. If none are found, the node falls back to generating a random obstacle along the following [layout](https://raw.github.com/qutas/qutas_lab_450/master/resources/occupancy_rand.png).


5. Open another terminal and run:

   ```rviz -d ~/EGH450/config/rviz_simulator_config.rviz```

6. Run demo waypoints with
- `demo_wp`: Performs a preset 4-waypoint task from with a range of configurable settings (waypoint radius, movement speed, _etc._)

   ``` rosrun spar_node demo_wp ```
  or run in arena:
```roslaunch spar_node demo_wp_roi.launch uav_name:=uavteam5```

- `demo_wp_roi`: Performs a preset4-waypoint task as above, with additional code to perform a diversion and continuation if a `geometry_msgs/PoseStamped` message is recieved.

   ```roslaunch spar_node demo_wp_roi.launch```

7. Run vision
```
rosrun depthai_publisher dai_publisher_yolov5_runner
rosrun depthai_publisher aruco_subscriber
roslaunch egh450_target_solvepnp pose_estimator.launch
```

## RQT for monitoring simulation workspace

1. Run `rqt` in a terminal and click plugins -> topic monitor
2. Click plugins -> mavros
3. In the mavros plugin pane click the settings button and set the namespace from /mavros to /uavasr


## Useful command for tmux
once a tmux session is running it can be attached to seperate shells via the following command:

``` tmux attach-session -t <name of tmux session>```

in the case of the gcs the 'name of tmux session' is 'cdrone'

## Another simulation environment is uavasr_emulator (is in the workspace of julians videos)
The [uavasr_emulator](https://github.com/qutas/uavasr_emulator) can be run with either of the following two commands, command 2 allows the specification of UAVNAME:

```
roslaunch uavasr_emulator emulator.launch
roslaunch uavasr_emulator emulator.launch uav_name:=UAVNAME
```



# Procedures for Autonomous Flight

## 1. SSH into the Onboard Pi
Open one terminal and SSH into the onboard Pi from the GCS.
```bash
ssh [username]@[IP address onboard]
# Example:
ssh gcsteam5@[IP address]
```

## 2. Run `run349`
This will execute all the required code for flight, including `roscore`.
```bash
run349
```

## 3. Open Additional Terminals
Open two more terminals and run the following commands:
```bash
rviz
rqt
```

## 4. Edit RQT
Configure RQT to display battery levels and the Mavros HUD.

## 5. Edit RVIZ
Set up RVIZ to display the following topics:
- `Mavros local_position/pose`
- Processed Aruco compressed image
- DepthAI publisher compressed images

## 6. Ensure Required Scripts are Running
Make sure the following are running in `tmux`:
- `control.launch`
- `environment.launch`
- `roscore`
- `rostopic echo` for the `mavros local_position/pose`
- DepthAI publisher
- Aruco subscriber scripts

## 7. Set Drone to 'Offboard' and Arm in RQT
Open RQT, set the drone to `offboard`, and then arm the drone.

## 8. Start Data Recording
In `tmux`, run the following command to start recording:
```bash
rosbag record -a
```

## 9. Run Autonomous Flight Code
In `tmux`, execute the code for autonomous flight:
```bash
rosrun spar_node [name of waypoint file]
```

## 10. Set 'Auto Land' and Prepare for Emergency
Return to RQT, set to `auto.land`, and be ready to arm in case of an emergency.

## 11. End of Flight
Once the flight is complete, if necessary, arm `auto.land` and stop data recording.

## 12. Conduct End of Flight Procedure
Terminate the session and shut down the system:
```bash
tmux kill-session
sudo shutdown now
```


---

# Adding a Submodule to a Git Repository

Adding a submodule allows you to keep another repository in your project, such as a library or a tool, while keeping its history separate from your main project. Here's a step-by-step guide on how to add a submodule to your repository.

### Prerequisites
Ensure you have Git installed on your system and you have a repository to which you want to add the submodule.

### Steps to Add a Submodule

1. **Clone Your Main Repository**
   If you haven't already, clone your main project repository:
   ```bash
   git clone https://github.com/your-username/your-repo.git
   cd your-repo
   ```

2. **Add the Submodule**
   Use the `git submodule add` command followed by the repository URL and the path within your project where you want the submodule:
   ```bash
   git submodule add <repository-url> <path/to/submodule>
   ```
   Replace `<repository-url>` with the URL of the repository you want to add as a submodule. Replace `<path/to/submodule>` with the path where you want to place the submodule in your project.

3. **Initialize the Submodule**
   After adding the submodule, you need to initialize it:
   ```bash
   git submodule init
   ```

4. **Fetch Data from the Submodule**
   Fetch all the data from the submodule:
   ```bash
   git submodule update
   ```

5. **Commit the Changes**
   Adding the submodule changes the `.gitmodules` file and your project's Git index. Commit these changes to your repository:
   ```bash
   git add .gitmodules <path/to/submodule>
   git commit -m "Add <submodule-name> as a submodule"
   ```

6. **Push the Changes**
   Finally, push the changes to your remote repository:
   ```bash
   git push
   ```

### Cloning a Project with Submodules
When cloning a repository that contains submodules, you should clone it using the `--recursive` option to automatically initialize and update each submodule:
```bash
git clone --recursive https://github.com/your-username/your-repo.git
```
If the repository was already cloned without submodules, you can load the submodules using:
```bash
git submodule update --init --recursive
```

### Conclusion
Submodules are a powerful feature that lets you manage separate projects within a single repository efficiently. They are particularly useful for including libraries, frameworks, or other dependencies that have their own lifecycle and versioning.
