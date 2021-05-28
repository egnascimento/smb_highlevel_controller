#include <cmath>
#include <algorithm>
#include <smb_highlevel_controller/SmbHighlevelController.hpp>

namespace smb_highlevel_controller {

////////////////////////////////////////////////////////////////////////////////////////////////////
SmbHighlevelController::SmbHighlevelController(ros::NodeHandle &nodeHandle) :
		_nodeHandle(nodeHandle),
		_subscriberQueueSize(10),
		_scanTopic("/scan"),
		_cmd_velTopic("/cmd_vel") {

	if (!readParameters()) {
		ROS_ERROR("Could not read parameters.");
		ros::requestShutdown();
	}

	// Publish registering of all topics
	_cmd_velPublisher = _nodeHandle.advertise<geometry_msgs::Twist>(_cmd_velTopic, 100);

	// Subscribe to all topics
	_scanSubscriber = _nodeHandle.subscribe(_scanTopic, _subscriberQueueSize, &SmbHighlevelController::scanCallback, this);
	_pclSubscriber = _nodeHandle.subscribe("/rslidar_points", 1, &SmbHighlevelController::pointcloudCallback, this);
}

////////////////////////////////////////////////////////////////////////////////////////////////////
SmbHighlevelController::~SmbHighlevelController() {
}

////////////////////////////////////////////////////////////////////////////////////////////////////
bool SmbHighlevelController::readParameters() {
	bool success = true;
	success &= _nodeHandle.getParam("/smb_highlevel_controller/scan_subscriber_topic_name", _scanTopic);
	success &= _nodeHandle.getParam("/smb_highlevel_controller/scan_subscriber_queue_size", _subscriberQueueSize);

	return success;
}

////////////////////////////////////////////////////////////////////////////////////////////////////
void SmbHighlevelController::pointcloudCallback(const sensor_msgs::PointCloud2 &msg) {
	ROS_INFO_STREAM_THROTTLE(2.0, "Pontos na nuvem 3D: " << msg.data.size());
}

////////////////////////////////////////////////////////////////////////////////////////////////////
void SmbHighlevelController::scanCallback(const sensor_msgs::LaserScan::ConstPtr &msg) {
	double min = msg->range_max;
	int sensor = 0;

	for (int i = 0; i < msg->ranges.size(); ++i) {
		if (msg->ranges[i] < min) {
			min = msg->ranges[i];
			sensor = i;
		}
	}

	_cmd_velMessage.linear.x = 0.2;
	_cmd_velMessage.linear.z = (sensor - 180.0) / 100.0;
	_cmd_velMessage.angular.z = (sensor - 180.0) / 100.0;

	_cmd_velPublisher.publish(_cmd_velMessage);

	ROS_INFO_STREAM_THROTTLE(2.0,
			"Medicao mais proxima: " << min << " no sensor " << sensor);
	ROS_INFO_STREAM_THROTTLE(2.0,
			"Atuacao de velocidade: LinX: " << _cmd_velMessage.linear.x << " LinZ: " << _cmd_velMessage.linear.z << " AngZ: " << _cmd_velMessage.angular.z);
}

}  // namespace smb_highlevel_controller
