#include "rclcpp/rclcpp.hpp"
// #include "std_msgs/msg/string.hpp"

#include "vision_comm/messages_robocup_ssl_detection.pb.h"
#include "vision_comm/messages_robocup_ssl_geometry.pb.h"
#include "vision_comm/messages_robocup_ssl_wrapper.pb.h"
#include "vision_comm/robocup_ssl_client.h"

#include "krssg_ssl_msgs/msg/ssl_detection_frame.hpp"
#include "krssg_ssl_msgs/msg/ssl_detection_ball.hpp"
#include "krssg_ssl_msgs/msg/ssl_detection_robot.hpp"

#include <sstream>
#include <ctime>
#include <google/protobuf/stubs/common.h>

using namespace std;

class VisionNode : public rclcpp::Node
{

private:
	rclcpp::Publisher<krssg_ssl_msgs::msg::SSLDetectionFrame>::SharedPtr publisher_;
	RoboCupSSLClient client_;
	bool use_grsim_vision_;
	std::shared_ptr<krssg_ssl_msgs::msg::SSLDetectionFrame> msg_;

public:
	VisionNode(bool& use_grsim_vision) : Node("vision_node"), use_grsim_vision_(use_grsim_vision) {
		rclcpp::QoS qos_profile = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort();
		publisher_ = create_publisher<krssg_ssl_msgs::msg::SSLDetectionFrame>("vision", qos_profile);

		GOOGLE_PROTOBUF_VERIFY_VERSION;
		// Initialize the SSL client
		int port = use_grsim_vision_ ? 10020 : 10006;
		client_ = RoboCupSSLClient(port);
		client_.open(true);
		RCLCPP_INFO(this->get_logger(), "Connected to %s", use_grsim_vision_ ? "grSim vision" : "ssl-vision");

		// Create the detection message object
    	msg_ = std::make_shared<krssg_ssl_msgs::msg::SSLDetectionFrame>();

		google::protobuf::ShutdownProtobufLibrary();
	}

	void run() {
		// Main loop
		while (rclcpp::ok())
		{
			processPacket();
			rclcpp::spin_some(shared_from_this());
		}
	}

private:
	void processPacket()
	{
		//RCLCPP_INFO(this->get_logger(), " ### processPacket ###");
		SSL_WrapperPacket packet;
	
		if (client_.receive(packet))
		{
			//RCLCPP_INFO(this->get_logger(), "Receive package");

			if (packet.has_detection())
			{
				const auto &detection = packet.detection();
				msg_->frame_number = detection.frame_number();
				msg_->t_capture = detection.t_capture();
				msg_->t_sent = detection.t_sent();
				msg_->camera_id = detection.camera_id();

				processBalls(detection);
				processRobotsBlue(detection);
				processRobotsYellow(detection);

				
				//RCLCPP_INFO(this->get_logger(), "Publishing message");
				publisher_->publish(*msg_);
			}

			// see if packet contains geometry data:
			//  right now, doing nothing for geometry data.
			if (packet.has_geometry())
			{
				const SSL_GeometryData &geom = packet.geometry();
				int calib_n = geom.calib_size();

				for (int i = 0; i < calib_n; i++)
				{
					const SSL_GeometryCameraCalibration &calib = geom.calib(i);
					printf("Camera Calibration...\n");
					printf("Camera ID: %d\n", calib.camera_id());
					printf("Focal Length: %f\n", calib.focal_length());
					printf("Principal Point X: %f\n", calib.principal_point_x());
					printf("Principal Point Y: %f\n", calib.principal_point_y());
				}
			}
		}
	}

	void processBalls(const SSL_DetectionFrame &detection)
	{
		msg_->balls.clear();
		for (int i = 0; i < detection.balls_size(); i++)
		{
			const auto &ball = detection.balls(i);
			auto ball_msg = std::make_shared<krssg_ssl_msgs::msg::SSLDetectionBall>();
			ball_msg->confidence = ball.confidence();
			ball_msg->area = ball.area();
			ball_msg->x = ball.x();
			ball_msg->y = ball.y();
			ball_msg->z = ball.z();
			ball_msg->pixel_x = ball.pixel_x();
			ball_msg->pixel_y = ball.pixel_y();
			msg_->balls.push_back(*ball_msg);
			//RCLCPP_INFO(this->get_logger(), "added balls to message");
		}
	}

	void processRobotsBlue(const SSL_DetectionFrame &detection)
	{
		msg_->robots_blue.clear();
		for (int i = 0; i < detection.robots_blue_size(); i++)
		{
			const auto &robot = detection.robots_blue(i);
			auto bot_msg = std::make_shared<krssg_ssl_msgs::msg::SSLDetectionRobot>();
			bot_msg->confidence = robot.confidence();
			bot_msg->robot_id = robot.robot_id();
			bot_msg->x = robot.x();
			bot_msg->y = robot.y();
			bot_msg->orientation = robot.orientation();
			bot_msg->pixel_x = robot.pixel_x();
			bot_msg->pixel_y = robot.pixel_y();
			bot_msg->height = robot.height();
			msg_->robots_blue.push_back(*bot_msg);
			// RCLCPP_INFO(this->get_logger(), "added robot blue to message");
		}
	}

	void processRobotsYellow(const SSL_DetectionFrame &detection)
	{
		msg_->robots_yellow.clear();
		for (int i = 0; i < detection.robots_yellow_size(); i++)
		{
			const auto &robot = detection.robots_yellow(i);
			auto bot_msg = std::make_shared<krssg_ssl_msgs::msg::SSLDetectionRobot>();
			bot_msg->confidence = robot.confidence();
			bot_msg->robot_id = robot.robot_id();
			bot_msg->x = robot.x();
			bot_msg->y = robot.y();
			bot_msg->orientation = robot.orientation();
			bot_msg->pixel_x = robot.pixel_x();
			bot_msg->pixel_y = robot.pixel_y();
			bot_msg->height = robot.height();
			msg_->robots_yellow.push_back(*bot_msg);
			// RCLCPP_INFO(this->get_logger(), "added robot yellow to message");
		}
	}
};

int main(int argc, char **argv)
{
	rclcpp::init(argc, argv);

	// by default, connects to SSLVision port
	// if a non-zero number is passed as argument, connects to grsim vision port.
	bool use_grsim_vision = (argc > 1 && std::atoi(argv[1]) != 0);

	auto node = std::make_shared<VisionNode>(use_grsim_vision);
	node->run();

	// this line broke the execution throwing an exception with std::bad_weak_ptr
	//rclcpp::spin(node);

	rclcpp::shutdown();

	return 0;
}
