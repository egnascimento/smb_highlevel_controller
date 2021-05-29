#pragma once

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <string.h>
#include <sensor_msgs/PointCloud2.h>
#include "geometry_msgs/Twist.h"
#include "visualization_msgs/Marker.h"

namespace smb_highlevel_controller {

/**
 * Class containing the SMB Highlevel Controller
 */
class SmbHighlevelController {
public:
	SmbHighlevelController(ros::NodeHandle &nodeHandle);
	virtual ~SmbHighlevelController();

private:
	bool readParameters();
	void scanCallback(const sensor_msgs::LaserScan::ConstPtr &msg);
	void pointcloudCallback(const sensor_msgs::PointCloud2 &msg);
	void publishMarker(int x, int y, int z);

	ros::NodeHandle _nodeHandle;
	ros::Subscriber _scanSubscriber;
	ros::Subscriber _pclSubscriber;
	ros::Publisher _cmd_velPublisher;
	ros::Publisher _markerPublisher;

	std::string _scanTopic;
	std::string _cmd_velTopic;

	float _pidP;
	geometry_msgs::Twist _cmd_velMessage;
	int _subscriberQueueSize;

};

}  // namespace smb_highlevel_controller
