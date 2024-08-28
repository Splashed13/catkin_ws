#pragma once

#include <ros/ros.h>

#include <nav_msgs/OccupancyGrid.h>

#include <vector>
#include <string>

enum obstacles_type_t {
	OBS_NONE = 0,
	OBS_SQUARE,
	OBS_CIRCLE
};

struct obstacles_t {
	obstacles_type_t type;
	int size;
	int x;
	int y;
};

class AreaMap {
	private:
		ros::NodeHandle nh_;
		ros::Publisher pub_map_;

		nav_msgs::OccupancyGrid msg_out_;

		std::string topic_map_;
		std::string param_frame_id_;
		int param_map_width_;
		int param_map_height_;
		double param_map_resolution_;
		bool param_map_boarder_;
		int param_num_obs_;

		std::vector<obstacles_t> obs_;

	public:
		AreaMap( void );

		~AreaMap( void );

		void generate_map( void );
};
