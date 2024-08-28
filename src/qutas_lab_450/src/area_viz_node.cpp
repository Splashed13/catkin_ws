#include <ros/ros.h>
#include <area_viz/area_viz.h>

int main(int argc, char** argv) {
	ros::init(argc, argv, "area_viz_cpp");
	AreaViz av;

	ros::spin();

	return 0;
}
