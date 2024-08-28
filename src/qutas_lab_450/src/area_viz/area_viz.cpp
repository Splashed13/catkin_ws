#include <ros/ros.h>

#include <area_viz/area_viz.h>

#include <nav_msgs/OccupancyGrid.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

AreaViz::AreaViz() :
	nh_("~"),
	topic_grid_("grid"),
	topic_markers_("markers"),
	topic_marker_namespace_("area_viz"),
	param_marker_height_(1.0),
	param_marker_alpha_(1.0),
	param_marker_red_(0.0),
	param_marker_green_(0.0),
	param_marker_blue_(0.0) {

	//Load node parameters
	nh_.param("topic_grid", topic_grid_, topic_grid_);
	nh_.param("topic_markers", topic_markers_, topic_markers_);

	nh_.param("marker/namespace", topic_marker_namespace_, topic_marker_namespace_);
	nh_.param("marker/height", param_marker_height_, param_marker_height_);
	nh_.param("marker/color/alpha", param_marker_alpha_, param_marker_alpha_);
	nh_.param("marker/color/red", param_marker_red_, param_marker_red_);
	nh_.param("marker/color/green", param_marker_green_, param_marker_green_);
	nh_.param("marker/color/blue", param_marker_blue_, param_marker_blue_);

	//Setup publisher and subscriber
	pub_markers_ = nh_.advertise<visualization_msgs::MarkerArray>( topic_markers_, 1, true );
	sub_grid_ = nh_.subscribe<nav_msgs::OccupancyGrid>( topic_grid_, 1, &AreaViz::callback_grid, this );

	//Done
	ROS_INFO("Publishing map vizualization");
}

AreaViz::~AreaViz() {
}

void AreaViz::callback_grid(const nav_msgs::OccupancyGrid::ConstPtr& msg_in) {
	msg_out_.markers.clear();

	for( int k = 0; k < msg_in->data.size(); k++ ) {
		if( msg_in->data[k] == 100 ) {
			visualization_msgs::Marker marker;

			marker.header.stamp = msg_in->header.stamp;
			marker.header.frame_id = msg_in->header.frame_id;
			marker.ns = topic_marker_namespace_;
			marker.id = k;
			marker.type = marker.CUBE;
			marker.action = marker.ADD;
			marker.lifetime = ros::Duration(0);
			marker.frame_locked = true;

			int i = k % msg_in->info.width;
			int j = (k - i) / msg_in->info.height;

			double dx = (i * msg_in->info.resolution) + (msg_in->info.resolution / 2);
			double dy = (j * msg_in->info.resolution) + (msg_in->info.resolution / 2);
			double dz = param_marker_height_ / 2;

			marker.pose.position.x = msg_in->info.origin.position.x + dx;
			marker.pose.position.y = msg_in->info.origin.position.y + dy;
			marker.pose.position.z = msg_in->info.origin.position.z + dz;
			marker.pose.orientation.w = 1.0;
			marker.pose.orientation.x = 0.0;
			marker.pose.orientation.y = 0.0;
			marker.pose.orientation.z = 0.0;

			marker.scale.x = msg_in->info.resolution;
			marker.scale.y = msg_in->info.resolution;
			marker.scale.z = param_marker_height_;

			marker.color.r = param_marker_red_;
			marker.color.g = param_marker_green_;
			marker.color.b = param_marker_blue_;
			marker.color.a = param_marker_alpha_;

			msg_out_.markers.push_back( marker );
		}
	}

	pub_markers_.publish( msg_out_ );
}
