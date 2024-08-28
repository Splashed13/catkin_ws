#pragma once

#include <ros/ros.h>

#include <nav_msgs/OccupancyGrid.h>
#include <visualization_msgs/MarkerArray.h>

#include <string>

class AreaViz {
	private:
		ros::NodeHandle nh_;
		ros::Subscriber sub_grid_;
		ros::Publisher pub_markers_;

		std::string topic_grid_;
		std::string topic_markers_;
		std::string topic_marker_namespace_;
		double param_marker_height_;
		double param_marker_alpha_;
		double param_marker_red_;
		double param_marker_green_;
		double param_marker_blue_;

		visualization_msgs::MarkerArray msg_out_;

	public:
		AreaViz( void );

		~AreaViz( void );

		void callback_grid(const nav_msgs::OccupancyGrid::ConstPtr& msg_in);
};
