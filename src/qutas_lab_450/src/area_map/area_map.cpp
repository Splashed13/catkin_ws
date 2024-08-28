#include <ros/ros.h>

#include <area_map/area_map.h>

#include <nav_msgs/OccupancyGrid.h>

#include <stdlib.h>     /* srand, rand */
#include <time.h>       /* time */

AreaMap::AreaMap() :
	nh_("~"),
	topic_map_("grid"),
	param_frame_id_("map"),
	param_map_width_(10),
	param_map_height_(10),
	param_map_resolution_(0.1),
	param_map_boarder_(false),
	param_num_obs_(0) {

	//Load node parameters
	nh_.param("frame_id", param_frame_id_, param_frame_id_);

	nh_.param("map/width", param_map_width_, param_map_width_);
	nh_.param("map/height", param_map_height_, param_map_height_);
	nh_.param("map/resolution", param_map_resolution_, param_map_resolution_);
	nh_.param("map/boarder", param_map_boarder_, param_map_boarder_);

	int i = 0;
	std::string obs_type;
	while( nh_.getParam("obstacles/obs_" + std::to_string(i) + "/type", obs_type) ) {
		obstacles_t obs;

		ROS_INFO("Loading obstacle %i (%s)", i, obs_type.c_str());

		if(obs_type == "square") {
			obs.type = OBS_SQUARE;
		} else if(obs_type == "circle") {
			obs.type = OBS_CIRCLE;
		} else {
			obs.type = OBS_NONE;
			ROS_ERROR("Unknown obstacle type: %s", obs_type.c_str());
		}

		if( (obs.type != OBS_NONE) &&
			nh_.getParam("obstacles/obs_" + std::to_string(i) + "/size", obs.size) &&
			nh_.getParam("obstacles/obs_" + std::to_string(i) + "/position/x", obs.x) &&
			nh_.getParam("obstacles/obs_" + std::to_string(i) + "/position/y", obs.y) ) {

			obs_.push_back(obs);
		}

		i++;
	}

	if(i == 0) {
		srand (time(NULL));

		obstacles_t obs;
		obs.type = OBS_SQUARE;
		obs.size = 3;
		int axes_div = 6;
		//place other axes at random -100% to +100%
		double p = 2*(((rand() % 101) / 100.0) - 0.5);
		//p=-1.0;

		nh_.param("obstacles/size", obs.size, obs.size);
		nh_.param("obstacles/divisor", axes_div, axes_div);

		//Place obstacle along one of the axes at random
		if(rand() % 2) {
			obs.x = param_map_width_ / axes_div;
			obs.y = p * param_map_width_ / axes_div;
		} else {
			obs.x = p * param_map_width_ / axes_div;
			obs.y = param_map_height_ / axes_div;
		}

		obs.x += param_map_width_ / 2;
		obs.y += param_map_height_ / 2;

		ROS_INFO("No obstacles define, placing random obstacle (s:%i;d:%i;p:%0.2f)", obs.size, axes_div, p);
		obs_.push_back(obs);
	}

	//Setup publisher
	pub_map_ = nh_.advertise<nav_msgs::OccupancyGrid>("grid", 1, true);

	//Generate Map
	generate_map();

	ROS_INFO("Generated map with %i obstacles", (int)obs_.size());

	//Publish map data (latched)
	pub_map_.publish(msg_out_);
}

AreaMap::~AreaMap() {
}

void AreaMap::generate_map( void ) {
	ros::Time stamp = ros::Time::now();

	msg_out_.header.frame_id = param_frame_id_;
	msg_out_.header.stamp = stamp;

	msg_out_.info.map_load_time = stamp;
	msg_out_.info.resolution = param_map_resolution_;
	msg_out_.info.width = param_map_width_;
	msg_out_.info.height = param_map_height_;

	msg_out_.info.origin.position.x = -param_map_resolution_ * (param_map_width_ / 2);
	msg_out_.info.origin.position.y = -param_map_resolution_ * (param_map_height_ / 2);
	msg_out_.info.origin.position.z = 0.0;
	msg_out_.info.origin.orientation.w = 1.0;
	msg_out_.info.origin.orientation.x = 0.0;
	msg_out_.info.origin.orientation.y = 0.0;
	msg_out_.info.origin.orientation.z = 0.0;

	//Load in base map
	for( int i = 0; i < param_map_width_; i++ ) {
		for( int j = 0; j < param_map_height_; j++ ) {
			int8_t spot = 0;

			//Draw in the boarder
			if( param_map_boarder_ &&
				( ( i == 0 ) || (j == 0 ) || ( i == ( param_map_width_ - 1 ) ) || ( j == ( param_map_height_ - 1 ) ) ) ) {
				spot = 100;
			}

			msg_out_.data.push_back(spot);
		}
	}

	//Load in obstacles
	for( int k = 0; k < obs_.size(); k++ ) {
		switch(obs_[k].type) {
			case OBS_SQUARE: {
				for( int y = -obs_[k].size; y < obs_[k].size + 1; y++ ) {
					for( int x = -obs_[k].size; x < obs_[k].size + 1; x++ ) {
						int dx = obs_[k].x + x;
						int dy = obs_[k].y + y;

						if ( (dx >= 0) &&
							 (dx < param_map_width_) &&
							 (dy >= 0) &&
							 (dy < param_map_height_) ) {
								msg_out_.data[dx + (dy*param_map_width_)] = 100;
						}
					}
				}

				break;
			}
			case OBS_CIRCLE: {
				for( int y = -obs_[k].size; y < obs_[k].size + 1; y++ ) {
					for( int x = -obs_[k].size; x < obs_[k].size + 1; x++ ) {
						int dx = obs_[k].x + x;
						int dy = obs_[k].y + y;
						int r = obs_[k].size;

						if ( (dx >= 0) &&
							 (dx < param_map_width_) &&
							 (dy >= 0) &&
							 (dy < param_map_height_) &&
							 ( (x*x + y*y) <= (r*r + r*0.25) ) ) {
								msg_out_.data[dx + (dy*param_map_width_)] = 100;
						}
					}
				}

				break;
			}
			default: {
				ROS_ERROR("Cannot load obstacle: %i", k);
			}
		}
	}
}
