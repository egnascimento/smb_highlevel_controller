#include <cmath>
#include <algorithm>
#include <smb_highlevel_controller/SmbHighlevelController.hpp>

namespace smb_highlevel_controller {

////////////////////////////////////////////////////////////////////////////////////////////////////
SmbHighlevelController::SmbHighlevelController(ros::NodeHandle &nodeHandle) :
		_nodeHandle(nodeHandle),
		_subscriberQueueSize(10),
		_scanTopic("/scan"),
		_cmd_velTopic("/cmd_vel"),
		_pidP(1)
		{

	if (!readParameters()) {
		ROS_ERROR("Could not read parameters.");
		ros::requestShutdown();
	}

	// Publish registering of all topics
	_cmd_velPublisher = _nodeHandle.advertise<geometry_msgs::Twist>(_cmd_velTopic, 100);
	_markerPublisher = _nodeHandle.advertise<visualization_msgs::Marker>("/marker", 1);

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
	success &= _nodeHandle.getParam("/smb_highlevel_controller/pid_p", _pidP);

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
	_cmd_velMessage.linear.z = ((sensor - 180.0) / 100.0) * _pidP;
	_cmd_velMessage.angular.z = ((sensor - 180.0) / 100.0) * _pidP;
	_cmd_velPublisher.publish(_cmd_velMessage);

	// Command used for cmd_vel testing: rostopic pub /cmd_vel geometry_msgs/Twist -r 10 -- '[0.5, 0.0, -0.5]' '[0.0, 0.0, -0.5]'

	// Publish estimated target
	publishMarker(min, 0, 0);


	ROS_INFO_STREAM_THROTTLE(2.0, "Medicao mais proxima: " << min << " no sensor " << sensor);
	ROS_INFO_STREAM_THROTTLE(2.0, "Velocidade: LinX: " << _cmd_velMessage.linear.x << " LinZ: "
													   << _cmd_velMessage.linear.z << " AngZ: "
													   << _cmd_velMessage.angular.z);
}

////////////////////////////////////////////////////////////////////////////////////////////////////
void SmbHighlevelController::publishMarker(int x, int y, int z) {

	visualization_msgs::Marker marker;
	marker.header.frame_id = "base_link";
	marker.header.stamp = ros::Time();

	marker.ns = "marker";

	marker.type = visualization_msgs::Marker::CYLINDER;
	marker.action = visualization_msgs::Marker::ADD;

	marker.pose.position.x = x;
	marker.pose.position.y = y;
	marker.pose.position.z = z;

	marker.pose.orientation.x = 0;
	marker.pose.orientation.y = 0;
	marker.pose.orientation.z = 0;
	marker.pose.orientation.w = 0;

	marker.scale.x = 1;
	marker.scale.y = 1;
	marker.scale.z = 1;

	marker.color.a = 1.0;
	marker.color.r = 255.0;
	marker.color.g = 69.0;
	marker.color.b = 0.0;

	_markerPublisher.publish(marker);
}

}  // namespace smb_highlevel_controller
