#pragma once

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <string.h>
#include <sensor_msgs/PointCloud2.h>
#include "geometry_msgs/Twist.h"

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

	ros::NodeHandle _nodeHandle;
	ros::Subscriber _scanSubscriber;
	std::string _scanTopic;
	std::string _cmd_velTopic;
	geometry_msgs::Twist _cmd_velMessage;
	ros::Publisher _cmd_velPublisher;
	int _subscriberQueueSize;

	ros::Subscriber _pclSubscriber;
};

}  // namespace smb_highlevel_controller
