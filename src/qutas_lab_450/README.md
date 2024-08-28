# qutas_lab_450
This package contains a set of nodes and launch files that are used for the final testing of the EGH450 unit at QUT.

## Launch Files
The following launch files are provided as a meants of providing a quick-start for the project:
- `environment.launch`:
	- Launches the following nodes:
		- `vrpn_client/vrpn_client_node`: The VRPN Client interface that connects to the VICON system
		- `qutas_lab_450/area_map_node`: A occupancy grid generator that generates a random obstacle in the search area
		- `qutas_lab_450/area_viz_node`: Provides a 3D marker visualization of the flight environment.
	- Can be launched with: `roslaunch qutas_lab_450 environment.launch`
	- Can be tested without the VICON nodes by setting the `vicon` parameter: `roslaunch qutas_lab_450 environment.launch vicon:=false`
- `dashboard.launch`:
	- Launches the following nodes:
		- `rqt_gui/rqt_gui`: Opens up the RQT control panel interface
	- Loads in the `config/dashboard.perspective` package file containing the following plugins:
		- `rqt_mavros_gui`: A GUI for common MAVROS commands
		- `rqt_robin_gcs`: A GUI for common commands for the robin firmware
		- `rqt_generic_hud`: A generic display for monitoring arbitrary message values
	- Can be copied and customised to start your own preset GUI dashboard
- `example_flight_control.launch`
	- Launches the following nodes:
		- `mavel/mavel_node`: Performs UAV position control
		- `odom_sync/odom_sync_node`: Syncronizer tool to combine Pose and Twist data into an Odometry message (for mavel)
		- `topic_tools/drop`: Set to drop a specific amount of pose messages (from the VICON interface) such that low-baud telemetry systems don't get overloaded when sending pose estimates to the flight controller
- `takeoff_test.launch`: Starts a connection to contrail to command the UAV to raise to 1m and then return to land. Good performing the first autonomous test flight.
- `assessment_sim_2021.launch`: Starts the UAVASR emulator, spar, the assorted QUTAS lab environment nodes to simulate the assessment conditions (occupancy grid generation an visualisation).

## Advanced usage of area_map_node
There are two modes available for defining obstacles within the flight area when using the `qutas_lab_450/area_map_node` node. The `environment.launch` file automatically loads in the file `launch/map_layout.yaml`. In this file, the obstacles are defined in the `obstacles:` parameters.

When the node first loads, it checks to see if any obstacles have been statically defined. If some are found, these obstacles are used to generate the desired layout. If none are found, the node falls back to generating a random obstacle along the following layout:
![Obstacle Random Placement](https://raw.github.com/qutas/qutas_lab_450/master/resources/occupancy_rand.png)


## Contrail Movement Scripts
The following contrail scripts are included for use with the `dispatcher`:
- `config/takeoff_test.yaml`: An example of a simple flight to hover to 1m, then land (used for `takeoff_test.launch`).
- `config/takeoff_quick.yaml`: An example of how to perform a takeoff to quickly escape the ground effect (**does not land**)
