#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

using namespace std::chrono_literals;

class Challenge02 : public rclcpp::Node
{
public:
	Challenge02() : Node("challenge_02"){
		path_publisher_ = this->create_publisher<nav_msgs::msg::Path>("path", 10);
		timer_ = this->create_wall_timer(
			100ms, std::bind(&Challenge02::timer_callback, this));
		this->time_ = 0.0;
		subscription_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
				"topic", 10, std::bind(&Challenge02::topic_callback, this, std::placeholders::_1));
	}

private:
	rclcpp::TimerBase::SharedPtr timer_;
	rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_publisher_;
	rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr subscription_;
	nav_msgs::msg::Path path_msg_;
	void timer_callback();
	void topic_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
	double time_;
	double x0_,y0_,z0_; // Entry position coordinates
	double vx0_,vy0_,vz0_; // Entry velocity vector components
};

void Challenge02::topic_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg){
	// Reset the time
	this->time_ = 0.0;

	// Get the entry position from the message
	this->x0_ = msg->pose.position.x; // Entry position, x-coordinate
	this->y0_ = msg->pose.position.y; // Entry position, y-coordinate
	this->z0_ = msg->pose.position.z; // Entry position, z-coordinate

	// The orientation member of geometry_msgs/Pose is of type 
	// geometry_msgs/Quaternion, but for simplicity the three first 
	// components are used to describe a velocity vector in this case.
	this->vx0_ = msg->pose.orientation.x; // Entry velocity vector, x-component
	this->vy0_ = msg->pose.orientation.y; // Entry velocity vector, y-component
	this->vz0_ = msg->pose.orientation.z; // Entry velocity vector, z-component
	RCLCPP_INFO(this->get_logger(), "New projectile at position (%0.3f,%0.3f,%0.3f", this->x0_, this->y0_, this->z0_);
	RCLCPP_INFO(this->get_logger(), "             with velocity (%0.3f,%0.3f,%0.3f", this->vx0_, this->vy0_, this->vz0_);
}

void Challenge02::timer_callback(){
	double x = this->x0_;
	double y = this->y0_;
	double z = this->z0_;
	//////////////////////////////////////////////////
	//////////// Start of contribution ///////////////
	//////////////////////////////////////////////////

	//TODO

	//////////////////////////////////////////////////
	///////////// End of contribution ////////////////
	//////////////////////////////////////////////////
	this->time_ += 0.1;
	if(!(z < 0)){
		auto pose_msg = geometry_msgs::msg::PoseStamped();
		pose_msg.header.stamp = this->now();
		pose_msg.header.frame_id = "map";
		pose_msg.pose.position.x = x;
		pose_msg.pose.position.y = y;
		pose_msg.pose.position.z = z;
		this->path_msg_.header.stamp = this->now();
		this->path_msg_.header.frame_id = "map";
		this->path_msg_.poses.push_back(pose_msg);
		RCLCPP_INFO(this->get_logger(), "Path:\tx: %0.3f\ty: %0.3f\tz: %0.3f", x, y, z);
		path_publisher_->publish(this->path_msg_);
	}
}

int main(int argc, char ** argv){
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<Challenge02>());
	rclcpp::shutdown();
	return 0;
}
