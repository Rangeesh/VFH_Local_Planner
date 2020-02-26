
#ifndef VFH_NODE_H_
#define VFH_NODE_H_

#include <vfh/vfh_algorithm.h>
#include <ros/ros.h>

#include <sensor_msgs/LaserScan.h>

#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Twist.h>

#include <nav_msgs/Odometry.h>


#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>

#define DEG2RAD(a) ((a) * M_PI / 180.0)

std::string scan_topic_  = "base_scan";
std::string odom_topic_  = "odom";

class VFH_node
{
public:
	VFH_node(ros::NodeHandle nh, ros::NodeHandle nh_private);
	~VFH_node();
	void update();
private:
	VFH_Algorithm *m_vfh;

	double m_cell_size;			// 100 mm
	int m_window_diameter;		// cells
	int m_sector_angle;			// in deg
	double m_safety_dist_0ms;
	double m_safety_dist_1ms;
	int m_max_speed;
	int m_max_speed_narrow_opening;
	int m_max_speed_wide_opening;
	int m_max_acceleration;
	int m_min_turnrate;
	int m_max_turnrate_0ms;
	int m_max_turnrate_1ms;
	double m_min_turn_radius_safety_factor;
	double m_free_space_cutoff_0ms;
	double m_obs_cutoff_0ms;
	double m_free_space_cutoff_1ms;
	double m_obs_cutoff_1ms;
	double m_weight_desired_dir;
	double m_weight_current_dir;

	double m_robot_radius;
	double m_robotVel;
    double m_laser_ranges[361][2];

	int chosen_speed,chosen_turnrate;

	// ros
    ros::NodeHandle nh_;
    ros::NodeHandle nh_private_;
    ros::Subscriber scan_subscriber_;
    ros::Subscriber odom_subscriber_;
    ros::Publisher  vel_publisher_;

    void scanCallback (const sensor_msgs::LaserScan::ConstPtr& scan_msg);
    void odomCallback (const nav_msgs::Odometry::ConstPtr& odom_msg);
};

#endif /* VFH_NODE_H_ */
