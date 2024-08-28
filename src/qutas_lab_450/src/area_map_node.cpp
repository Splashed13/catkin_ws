#include <ros/ros.h>
#include <area_map/area_map.h>

int main(int argc, char** argv) {
	ros::init(argc, argv, "area_map_cpp");
	AreaMap am;

	ros::spin();

	return 0;
}
