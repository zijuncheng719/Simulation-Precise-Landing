/****************************************************************************
 *
 * Copyright 2020 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 * list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its contributors
 * may be used to endorse or promote products derived from this software without
 * specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @brief Offboard control example
 * @file offboard_control.cpp
 * @addtogroup examples
 * @author Mickey Cowden <info@cowden.tech>
 * @author Nuno Marques <nuno.marques@dronesolutions.io>
 */

#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_control_mode.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <px4_msgs/msg/manual_control_setpoint.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp> // 包含必要的头文件
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/qos.hpp>
#include <stdint.h>

#include <chrono>
#include <iostream>

using namespace std::chrono;
using namespace std::chrono_literals;
using namespace px4_msgs::msg;

struct Coordinate {
    double x;
    double y;
    double z;
};

class OffboardControl : public rclcpp::Node
{
public:
	OffboardControl() : Node("offboard_control")
	{
		// 设置QoS为最佳努力
    	rclcpp::QoS qos(rclcpp::KeepLast(10));
    	qos.reliability(rclcpp::ReliabilityPolicy::BestEffort);

		// 创建参数描述符
		rcl_interfaces::msg::ParameterDescriptor descriptor1;
		descriptor1.description = "local home position based on NED earth-fixed frame";
		// 创建浮点数范围
		rcl_interfaces::msg::FloatingPointRange range1_x;
		range1_x.from_value = 0.0;
		range1_x.to_value = 10.0;
		range1_x.step = 0.1;
		rcl_interfaces::msg::FloatingPointRange range1_y;
		range1_y.from_value = -5.0;
		range1_y.to_value = 5.0;
		range1_y.step = 0.1;
		rcl_interfaces::msg::FloatingPointRange range1_z;
		range1_z.from_value = -10.0;
		range1_z.to_value = 0.0;
		range1_z.step = 0.1;
		// 将范围添加到描述符中
		descriptor1.floating_point_range = {range1_x};

		// rcl_interfaces::msg::ParameterDescriptor descriptor2;
		// descriptor2.description = "land height of 3 stage in the precise_landing based on NED earth-fixed frame";
		// rcl_interfaces::msg::FloatingPointRange range2_x;
		// range2_x.from_value = -10.0;
		// range2_x.to_value = 0.0;
		// range2_x.step = 0.1;
		// rcl_interfaces::msg::FloatingPointRange range2_y;
		// range2_y.from_value = -10.0;
		// range2_y.to_value = 0.0;
		// range2_y.step = 0.1;
		// rcl_interfaces::msg::FloatingPointRange range2_z;
		// range2_z.from_value = -10.0;
		// range2_z.to_value = 0.0;
		// range2_z.step = 0.1;
		// descriptor2.floating_point_range = {range2_x, range2_y, range2_z};

		// rcl_interfaces::msg::ParameterDescriptor descriptor3;
		// descriptor3.description = "the offset of aruco position based on FRD body-fixed frame";
		// rcl_interfaces::msg::FloatingPointRange range3_x;
		// range3_x.from_value = -0.5;
		// range3_x.to_value = 0.5;
		// range3_x.step = 0.01;
		// rcl_interfaces::msg::FloatingPointRange range3_y;
		// range3_y.from_value = -0.5;
		// range3_y.to_value = 0.5;
		// range3_y.step = 0.01;
		// rcl_interfaces::msg::FloatingPointRange range3_z;
		// range3_z.from_value = -0.5;
		// range3_z.to_value = 0.5;
		// range3_z.step = 0.01;
		// descriptor3.floating_point_range = {range3_x, range3_y, range3_z};

		// rcl_interfaces::msg::ParameterDescriptor descriptor4;
		// descriptor4.description = "the offset of aruco yaw based on FRD body-fixed frame";
		// rcl_interfaces::msg::FloatingPointRange range4_x;
		// range4_x.from_value = -0.5;
		// range4_x.to_value = 0.5;
		// range4_x.step = 0.01;
		// descriptor4.floating_point_range = {range4_x};

		// rcl_interfaces::msg::ParameterDescriptor descriptor5;
		// descriptor5.description = "the control parameter K of yaw";
		// rcl_interfaces::msg::FloatingPointRange range5_x;
		// range5_x.from_value = -1.0;
		// range5_x.to_value = 1.0;
		// range5_x.step = 0.01;
		// descriptor5.floating_point_range = {range5_x};

		// rcl_interfaces::msg::ParameterDescriptor descriptor6;
		// descriptor6.description = "the allowed error of xyz";
		// rcl_interfaces::msg::FloatingPointRange range6_x;
		// range6_x.from_value = -3.0;
		// range6_x.to_value = 3.0;
		// range6_x.step = 0.05;
		// descriptor6.floating_point_range = {range6_x};

		// rcl_interfaces::msg::ParameterDescriptor descriptor7;
		// descriptor7.description = "the allowed error of yaw";
		// rcl_interfaces::msg::FloatingPointRange range7_x;
		// range7_x.from_value = -0.3;
		// range7_x.to_value = 0.3;
		// range7_x.step = 0.01;
		// descriptor7.floating_point_range = {range7_x};

		// rcl_interfaces::msg::ParameterDescriptor descriptor8;
		// descriptor8.description = "the allowed visual error of xyz";
		// rcl_interfaces::msg::FloatingPointRange range8_x;	
		// range8_x.from_value = -2.0;
		// range8_x.to_value = 2.0;
		// range8_x.step = 0.05;
		// descriptor8.floating_point_range = {range8_x};

		// rcl_interfaces::msg::ParameterDescriptor descriptor9;
		// descriptor9.description = "the allowed visual error of yaw";
		// rcl_interfaces::msg::FloatingPointRange range9_x;
		// range9_x.from_value = -0.2;
		// range9_x.to_value = 0.2;
		// range9_x.step = 0.01;
		// descriptor9.floating_point_range = {range9_x};

		// rcl_interfaces::msg::ParameterDescriptor descriptor10;
		// descriptor10.description = "the optional yaw of local home";
		// rcl_interfaces::msg::FloatingPointRange range10_x;
		// range10_x.from_value = -0.2;
		// range10_x.to_value = 0.2;
		// range10_x.step = 0.01;
		// descriptor10.floating_point_range = {range10_x};
        // Declare parameters with default values,based on the NED earth-fixed frame
        this->declare_parameter("local_home", std::vector<double>{1.5, 0.3, -5.0});
		this->declare_parameter("land_height", std::vector<double>{-5.0, -3.0, -1.0});
		this->declare_parameter("offset_aruco_position_frd", std::vector<double>{0.0, 0.0, 0.0});
		this->declare_parameter("offset_aruco_yaw_frd", 0.0);
		this->declare_parameter("param_yaw", 0.3);
        this->declare_parameter("allow_err_xyz", 1.0);
        this->declare_parameter("allow_err_yaw", 0.2);
        this->declare_parameter("allow_visual_err_xyz", 0.25);
        this->declare_parameter("allow_visual_err_yaw", 0.15);
		this->declare_parameter("local_home_yaw", 0.0);
		this->declare_parameter("loss_of_aruco_protection_time", 10);
        // this->declare_parameter("local_home", std::vector<double>{1.5, 0.3, -5.0}, descriptor1);
		// this->declare_parameter("land_height", std::vector<double>{-5.0, -3.0, -1.0}, descriptor2);
		// this->declare_parameter("offset_aruco_position_frd", std::vector<double>{0.0, 0.0, 0.0}, descriptor3);
		// this->declare_parameter("offset_aruco_yaw_frd", 0.0, descriptor4);
		// this->declare_parameter("param_yaw", 0.3, descriptor5);
        // this->declare_parameter("allow_err_xyz", 1.0, descriptor6);
        // this->declare_parameter("allow_err_yaw", 0.2, descriptor7);
        // this->declare_parameter("allow_visual_err_xyz", 0.25, descriptor8);
        // this->declare_parameter("allow_visual_err_yaw", 0.15, descriptor9);
		// this->declare_parameter("local_home_yaw", 0.0, descriptor10);

		offboard_control_mode_publisher_ = this->create_publisher<OffboardControlMode>("/fmu/in/offboard_control_mode", 10);
		trajectory_setpoint_publisher_ = this->create_publisher<TrajectorySetpoint>("/fmu/in/trajectory_setpoint", 10);
		vehicle_command_publisher_ = this->create_publisher<VehicleCommand>("/fmu/in/vehicle_command", 10);
		aruco_subscription_ = this->create_subscription<geometry_msgs::msg::PoseStamped>("/aruco_single/pose", 10, std::bind(&OffboardControl::pose_callback, this, std::placeholders::_1));
		local_position_subscription_ = this->create_subscription<VehicleLocalPosition>("/fmu/out/vehicle_local_position", qos, std::bind(&OffboardControl::local_position_callback, this, std::placeholders::_1));
		vehicle_status_subscription_ = this->create_subscription<px4_msgs::msg::VehicleStatus>("/fmu/out/vehicle_status", qos, std::bind(&OffboardControl::vehicle_status_callback, this, std::placeholders::_1));
		vehicle_odometry_subscriber_ = this->create_subscription<px4_msgs::msg::VehicleOdometry>("/fmu/out/vehicle_odometry", qos, std::bind(&OffboardControl::vehicle_odometry_callback, this, std::placeholders::_1));
		manual_control_setpoint_subscriber_ = this->create_subscription<px4_msgs::msg::ManualControlSetpoint>("/fmu/out/manual_control_setpoint", qos, std::bind(&OffboardControl::manual_control_setpoint_callback, this, std::placeholders::_1));

        // Retrieve parameters
        this->get_parameter("local_home", local_home_);
		this->get_parameter("land_height", land_height_);
		this->get_parameter("offset_aruco_position_frd", offset_aruco_position_frd_);
		this->get_parameter("offset_aruco_yaw_frd", offset_aruco_yaw_frd_);
		this->get_parameter("param_yaw", param_yaw_);
        this->get_parameter("allow_err_xyz", allow_err_xyz_);
        this->get_parameter("allow_err_yaw", allow_err_yaw_);
        this->get_parameter("allow_visual_err_xyz", allow_visual_err_xyz_);
        this->get_parameter("allow_visual_err_yaw", allow_visual_err_yaw_);
		this->get_parameter("local_home_yaw", local_home_yaw_);
		this->get_parameter("loss_of_aruco_protection_time", loss_of_aruco_protection_time_);

		// Initialize the setpoint counter
		offboard_setpoint_counter_ = 0;
		auto timer_callback = [this]() -> void {

			if (offboard_setpoint_counter_ == 10) {
				// Change to Offboard mode after 10 setpoints
				this->publish_vehicle_command(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6, 0);

				// Arm the vehicle
				this->arm();
			}

			// offboard_control_mode needs to be paired with trajectory_setpoint
			publish_offboard_control_mode();
			publish_trajectory_setpoint();

			offboard_setpoint_counter_++;
			
		};
		timer_ = this->create_wall_timer(100ms, timer_callback);

		// Initialize the mode switch timer
		auto mode_switch_timer_callback = [this]() -> void {
            if (take_off != true && preperation_precise_landing != true && visual_precise_landing != true && workstep == 3)
            {
                // Land the vehicle
                this->publish_vehicle_command(VehicleCommand::VEHICLE_CMD_NAV_LAND, 0, 0, 0);

                // Wait for the vehicle to land
                std::this_thread::sleep_for(std::chrono::seconds(5));

                // Exit offboard mode (e.g., switch to manual mode)
                this->publish_vehicle_command(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 0, 0);

                // Disarm the vehicle if landed
                if (vehicle_status_.nav_state == px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_AUTO_LAND) {
                    this->publish_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 0, 0);
                }

				workstep = -1;
            }

			if (rc_aux1 > 0.5)
			{	// Exit offboard mode (switch to position mode)
				rc_aux1_down_cnt_ = 0;
				rc_aux1_up_cnt_++;
				if (rc_aux1_up_cnt_ == 1)
				{
					this->publish_vehicle_command(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 4, 4);
				}
			}
			if (rc_aux1 < -0.5)
			{	// Exit offboard mode (switch to altitude mode)
				rc_aux1_up_cnt_ = 0;
				rc_aux1_down_cnt_++;
				if (rc_aux1_down_cnt_ == 1)
				{
					this->publish_vehicle_command(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 4, 2);
				}
			}
			if (rc_aux2 < -0.5)
			{	// Exit offboard mode (switch to land mode)
				rc_aux2_up_cnt_ = 0;
				rc_aux2_down_cnt_++;
				if (rc_aux2_down_cnt_ == 1)
				{
					this->publish_vehicle_command(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 4, 5);
				}
			}
			if (rc_aux2 > 0.5 && vehicle_status_.nav_state == px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_POSCTL &&
				std::abs(rc_throttle) < 0.05 && std::abs(rc_yaw) < 0.05 && std::abs(rc_pitch) < 0.05 && std::abs(rc_roll) < 0.05
				)	
			{	// Exit position mode (switch to offboard mode)进入方式，在定点模式下，油门摇杆、偏航摇杆、俯仰摇杆、横滚摇杆都在中间位置，此时按下aux2开关，进入offboard模式
				rc_aux2_down_cnt_ = 0;
				rc_aux2_up_cnt_++;
				if (rc_aux2_up_cnt_ == 10)
				{
					this->publish_vehicle_command(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6, 0);
					// Arm the vehicle
					this->arm();
				}
			}
		};
		mode_switch_timer_ = this->create_wall_timer(20ms, mode_switch_timer_callback);
	}

	void arm();
	void disarm();

private:
	bool take_off = true;
	bool preperation_precise_landing = false;
	bool visual_precise_landing = false;
	bool loss_of_aruco_protection = false;
	u_int64_t loss_of_aruco_protection_time_;	//aruco码丢失保护时间
	u_int64_t protect_cnt_ = 0;					//aruco码丢失保护计数器
	double tar_x;
	double tar_y;
	double tar_yaw;
	double tar_z;
	double local_position_x, local_position_y, local_position_z;	//无人机坐标是相对于世界坐标系（ENU）而言
	double local_position_yaw;
	double aruco_position_x, aruco_position_y, aruco_position_z;	//aruco坐标是相对于相机坐标系而言
	double aruco_position_frd_yaw;
	Coordinate enu, ned;	//ENU坐标系和NED坐标系
	double delta_x;
	double delta_y;
	double delta_z;
	double delta_aruco_frd_yaw;
	geometry_msgs::msg::PointStamped aruco_position_ned;
	geometry_msgs::msg::PointStamped aruco_position_frd;
	geometry_msgs::msg::PointStamped aruco_velosity_ned;
	geometry_msgs::msg::PointStamped aruco_velosity_frd;
	geometry_msgs::msg::TransformStamped transform_stamped;
	geometry_msgs::msg::TransformStamped transform_velosity_stamped;
	geometry_msgs::msg::PointStamped delta_aruco_position_frd;

	tf2::Quaternion quat_tf;
	double roll, pitch, yaw;

	std::vector<double> local_home_;
	std::vector<double> land_height_;
	std::vector<double> offset_aruco_position_frd_;
	double offset_aruco_yaw_frd_;
	double param_yaw_;
	double local_home_yaw_;
	double allow_err_xyz_;
	double allow_err_yaw_;
	double allow_visual_err_xyz_;
	double allow_visual_err_yaw_;
	int workstep = -1;
	uint64_t aruco_pose_time_stamp_now, aruco_pose_time_stamp_last;

	px4_msgs::msg::VehicleStatus vehicle_status_;

	rclcpp::TimerBase::SharedPtr timer_;
	rclcpp::TimerBase::SharedPtr mode_switch_timer_;

	rclcpp::Publisher<OffboardControlMode>::SharedPtr offboard_control_mode_publisher_;
	rclcpp::Publisher<TrajectorySetpoint>::SharedPtr trajectory_setpoint_publisher_;
	rclcpp::Publisher<VehicleCommand>::SharedPtr vehicle_command_publisher_;
	rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr aruco_subscription_;
	rclcpp::Subscription<VehicleLocalPosition>::SharedPtr local_position_subscription_;
	rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr vehicle_status_subscription_;
	rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr vehicle_odometry_subscriber_;
	rclcpp::Subscription<px4_msgs::msg::ManualControlSetpoint>::SharedPtr manual_control_setpoint_subscriber_;

	uint64_t offboard_setpoint_counter_;   //!< counter for the number of setpoints sent
	uint64_t counter_;	//任务流程短暂计时器

	float rc_aux1, rc_aux2, rc_throttle, rc_yaw, rc_pitch, rc_roll;
	u_int64_t rc_aux1_down_cnt_, rc_aux2_down_cnt_;
	u_int64_t rc_aux1_up_cnt_, rc_aux2_up_cnt_;


	void publish_offboard_control_mode();
	void publish_trajectory_setpoint();
	void publish_vehicle_command(uint16_t command, double param1 = 0.0, double param2 = 0.0, double param3 = 0.0);

	// NED to ENU conversion
	Coordinate NEDtoENU(const Coordinate& ned) {
    	Coordinate enu;
    	enu.x = ned.y;
    	enu.y = ned.x;
    	enu.z = -ned.z;
    	return enu;
	}

	// ENU to NED conversion
	Coordinate ENUtoNED(const Coordinate& enu) {
	    Coordinate ned;
    	ned.x = enu.y;
    	ned.y = enu.x;
    	ned.z = -enu.z;
    	return ned;
	}

  	void pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) 
	{
		tf2::fromMsg(msg->pose.orientation, quat_tf);
		tf2::Matrix3x3(quat_tf).getRPY(roll, pitch, yaw);
		aruco_position_x = msg->pose.position.x;
		aruco_position_y = msg->pose.position.y;
		aruco_position_z = msg->pose.position.z;
		aruco_position_frd_yaw = yaw;

		aruco_position_frd.point.x = -msg->pose.position.y;		//此处又涉及相机坐标系到机体坐标系的转换
		aruco_position_frd.point.y = msg->pose.position.x;
		aruco_position_frd.point.z = msg->pose.position.z;

		aruco_position_frd.point.x = aruco_position_frd.point.x - offset_aruco_position_frd_[0];
		aruco_position_frd.point.y = aruco_position_frd.point.y - offset_aruco_position_frd_[1];
		aruco_position_frd.point.z = aruco_position_frd.point.z - offset_aruco_position_frd_[2];
		aruco_position_frd_yaw = aruco_position_frd_yaw - offset_aruco_yaw_frd_;
		aruco_position_ned = Position_Transform_From_FRD_To_NED(aruco_position_frd);

		delta_aruco_position_frd.point.x = aruco_position_frd.point.x;
		delta_aruco_position_frd.point.y = aruco_position_frd.point.y;
		delta_aruco_position_frd.point.z = aruco_position_frd.point.z;
		delta_aruco_frd_yaw = aruco_position_frd_yaw;

		aruco_velosity_frd.point.x = delta_aruco_position_frd.point.x;
		aruco_velosity_frd.point.y = delta_aruco_position_frd.point.y;
		aruco_velosity_frd.point.z = delta_aruco_position_frd.point.z;
		aruco_velosity_ned = Velosity_Transform_From_FRD_To_NED(aruco_velosity_frd);

		aruco_pose_time_stamp_now++;

		// RCLCPP_INFO(this->get_logger(), "aruco_velosity_frd = [%f, %f, %f]", aruco_velosity_frd.point.x, aruco_velosity_frd.point.y, aruco_velosity_frd.point.z);
		// RCLCPP_INFO(this->get_logger(), "aruco_velosity_ned = [%f, %f, %f]", aruco_velosity_ned.point.x, aruco_velosity_ned.point.y, aruco_velosity_ned.point.z);
		// RCLCPP_INFO(this->get_logger(), "aruco_position_frd = [%f, %f, %f]", aruco_position_frd.point.x, aruco_position_frd.point.y, aruco_position_frd.point.z);
		// RCLCPP_INFO(this->get_logger(), "aruco_position_ned = [%f, %f, %f]", aruco_position_ned.point.x, aruco_position_ned.point.y, aruco_position_ned.point.z);
    	// RCLCPP_INFO(this->get_logger(), "aruco_position_frd_yaw = %f", aruco_position_frd_yaw);
  	}

  	void local_position_callback(const VehicleLocalPosition::SharedPtr msg) 
	{
		local_position_x = msg->x; local_position_y = msg->y; local_position_z = msg->z; local_position_yaw = msg->heading;
		delta_x = std::abs(local_position_x - local_home_[0]);
		delta_y = std::abs(local_position_y - local_home_[1]);
		delta_z = std::abs(local_position_z - local_home_[2]);
  	}

    void vehicle_status_callback(const px4_msgs::msg::VehicleStatus::SharedPtr msg) {
        vehicle_status_ = *msg;
    }

	void vehicle_odometry_callback(const px4_msgs::msg::VehicleOdometry::SharedPtr msg)
	{
    	// Extract quaternion from the message
    	tf2::Quaternion q(
    	    msg->q[1],
    	    msg->q[2],
    	    msg->q[3],
    	    msg->q[0]);

    	transform_stamped.header.frame_id = "NED earth-fixed frame"; // NED frame
    	transform_stamped.child_frame_id = "FRD body frame";
    	transform_stamped.transform.translation.x = msg->position[0];
    	transform_stamped.transform.translation.y = msg->position[1];
    	transform_stamped.transform.translation.z = msg->position[2];
    	transform_stamped.transform.rotation.x = q[0];
    	transform_stamped.transform.rotation.y = q[1];
    	transform_stamped.transform.rotation.z = q[2];
    	transform_stamped.transform.rotation.w = q[3];

    	transform_velosity_stamped.header.frame_id = "NED earth-fixed frame"; // NED frame		
		transform_velosity_stamped.child_frame_id = "FRD body frame";
		transform_velosity_stamped.transform.translation.x = 0;
		transform_velosity_stamped.transform.translation.y = 0;
		transform_velosity_stamped.transform.translation.z = 0;
		transform_velosity_stamped.transform.rotation.x = q[0];
		transform_velosity_stamped.transform.rotation.y = q[1];
		transform_velosity_stamped.transform.rotation.z = q[2];
		transform_velosity_stamped.transform.rotation.w = q[3];
	}

	geometry_msgs::msg::PointStamped Position_Transform_From_FRD_To_NED(geometry_msgs::msg::PointStamped point_frd)
	{
    	// Example: Transform a point from FLU to NED
    	geometry_msgs::msg::PointStamped point_ned;

    	tf2::doTransform(point_frd, point_ned, transform_stamped);

		// RCLCPP_INFO(this->get_logger(), "local Point: [%f, %f, %f]", local_position_x, local_position_y, local_position_z);
    	// RCLCPP_INFO(this->get_logger(), "NED Point: [%f, %f, %f]", point_ned.point.x, point_ned.point.y, point_ned.point.z);
		return point_ned;
	}

	geometry_msgs::msg::PointStamped Velosity_Transform_From_FRD_To_NED(geometry_msgs::msg::PointStamped point_frd)
	{
    	// Example: Transform a point from FLU to NED
    	geometry_msgs::msg::PointStamped point_ned;

    	tf2::doTransform(point_frd, point_ned, transform_velosity_stamped);

		// RCLCPP_INFO(this->get_logger(), "local Point: [%f, %f, %f]", local_position_x, local_position_y, local_position_z);
    	// RCLCPP_INFO(this->get_logger(), "NED Point: [%f, %f, %f]", point_ned.point.x, point_ned.point.y, point_ned.point.z);
		return point_ned;
	}

    void manual_control_setpoint_callback(const px4_msgs::msg::ManualControlSetpoint::SharedPtr msg)
    {
		// 处理接收到的消息
		rc_aux1 = msg->aux1;
		rc_aux2 = msg->aux2;
		rc_throttle = msg->throttle;
		rc_yaw = msg->yaw;
		rc_pitch = msg->pitch;
		rc_roll = msg->roll;
    }
};

/**
 * @brief Send a command to Arm the vehicle
 */
void OffboardControl::arm()
{
	publish_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0, 0.0);

	RCLCPP_INFO(this->get_logger(), "Arm command send");
}

/**
 * @brief Send a command to Disarm the vehicle
 */
void OffboardControl::disarm()
{
	publish_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0, 0.0);

	RCLCPP_INFO(this->get_logger(), "Disarm command send");
}

/**
 * @brief Publish the offboard control mode.
 *        For this example, only position and altitude controls are active.
 */
void OffboardControl::publish_offboard_control_mode()
{
	OffboardControlMode msg{};
	msg.position = true;
	msg.velocity = false;
	msg.acceleration = false;
	msg.attitude = false;
	msg.body_rate = false;
	msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
	offboard_control_mode_publisher_->publish(msg);
}

/**
 * @brief Publish a trajectory setpoint
 *        For this example, it sends a trajectory setpoint to make the
 *        vehicle hover at 5 meters with a yaw angle of 180 degrees.
 */
void OffboardControl::publish_trajectory_setpoint()
{
	TrajectorySetpoint msg{};

	RCLCPP_INFO(this->get_logger(), "workstep = %d", workstep);
	if (take_off)
	{
		msg.position = {-1.5, -1.5, -5};
		msg.yaw = local_home_yaw_; // [-PI:PI]
		if (offboard_setpoint_counter_ > 100)
		{
			take_off = false;
			preperation_precise_landing = true;
			visual_precise_landing = false;
			// RCLCPP_INFO(this->get_logger(), "offboard_setpoint_counter_ = %lu, take_off to preparation_precision_land ing", offboard_setpoint_counter_);
		}
		RCLCPP_WARN(this->get_logger(), "take_off");
	}
	if (preperation_precise_landing)	//GPS引导至精准降落点附近
	{
		msg.position = {static_cast<float>(local_home_[0]), static_cast<float>(local_home_[1]), static_cast<float>(local_home_[2])};
		msg.yaw = local_position_yaw + param_yaw_ * aruco_position_frd_yaw;
		if (delta_x < allow_err_xyz_ && delta_y < allow_err_xyz_ && delta_z < allow_err_xyz_ && 
			std::abs(delta_aruco_frd_yaw) < allow_visual_err_yaw_)
		{
			counter_++;
			if (counter_ > 15)
			{
			take_off = false;
			preperation_precise_landing = false;
			visual_precise_landing = true;
			counter_ = 0;
			workstep = 0;
			}
		}
		RCLCPP_WARN(this->get_logger(), "preperation_precise_landing");
	}

	if (visual_precise_landing)	//视觉实现精准降落
	{
		switch (workstep)
		{
		case 0:
			if (loss_of_aruco_protection != true)	//保护，防止出现二维码消失，仍然有初速度的情况
			{
				tar_x = local_position_x + 1 * aruco_velosity_ned.point.x;
				tar_y = local_position_y + 1 * aruco_velosity_ned.point.y;
				tar_z = local_position_z + 0.8 * (delta_aruco_position_frd.point.z + land_height_[workstep]);
				msg.position = {static_cast<float>(tar_x), static_cast<float>(tar_y), static_cast<float>(tar_z)};
				msg.yaw = local_position_yaw + param_yaw_ * aruco_position_frd_yaw;
			}
			else
			{
				msg.position = {static_cast<float>(tar_x), static_cast<float>(tar_y), static_cast<float>(tar_z)};
			}
			if (std::abs(delta_aruco_position_frd.point.x) < allow_visual_err_xyz_*5 &&
				std::abs(delta_aruco_position_frd.point.y) < allow_visual_err_xyz_*5 &&
				std::abs(delta_aruco_position_frd.point.z + land_height_[workstep]) < allow_visual_err_xyz_*5 &&
				std::abs(delta_aruco_frd_yaw) < allow_visual_err_yaw_*5
				)
			{
				counter_++;
				if (counter_ > 10)
				{
				workstep = 2;
				counter_ = 0;
				}
			}break;
		case 1:
			if (loss_of_aruco_protection != true)	//保护，防止出现二维码消失，仍然有初速度的情况
			{
				tar_x = local_position_x + 1 * aruco_velosity_ned.point.x;
				tar_y = local_position_y + 1 * aruco_velosity_ned.point.y;
				tar_z = local_position_z + 0.8 * (delta_aruco_position_frd.point.z + land_height_[workstep]);
				msg.position = {static_cast<float>(tar_x), static_cast<float>(tar_y), static_cast<float>(tar_z)};
				msg.yaw = local_position_yaw + param_yaw_ * aruco_position_frd_yaw;
			}
			else
			{
				msg.position = {static_cast<float>(tar_x), static_cast<float>(tar_y), static_cast<float>(tar_z)};
			}
			if (std::abs(delta_aruco_position_frd.point.x) < allow_visual_err_xyz_*3 &&
				std::abs(delta_aruco_position_frd.point.y) < allow_visual_err_xyz_*3 &&
				std::abs(delta_aruco_position_frd.point.z + land_height_[workstep]) < allow_visual_err_xyz_*3 &&
				std::abs(delta_aruco_frd_yaw) < allow_visual_err_yaw_*3
				)
			{
				counter_++;
				if (counter_ > 10)
				{
				workstep = 2;
				counter_ = 0;
				}
			}
			
			// counter_++;
			// if (counter_ > 15)
			// {
			// take_off = false;
			// preparation_precision_land = false;
			// precision_land = true;
			// counter_ = 0;
			// }
			break;
		case 2:
			if (true)
			{	// Exit offboard mode (switch to position mode)
				rc_aux1_down_cnt_ = 0;
				rc_aux1_up_cnt_++;
				if (rc_aux1_up_cnt_ == 1)
				{
					this->publish_vehicle_command(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 4, 4);
					RCLCPP_WARN(this->get_logger(), "position mode");
					
				}
			}
			if (loss_of_aruco_protection != true)	//保护，防止出现二维码消失，仍然有初速度的情况
			{
				tar_x = local_position_x + 1 * aruco_velosity_ned.point.x;
				tar_y = local_position_y + 1 * aruco_velosity_ned.point.y;
				tar_z = local_position_z + 0.8 * (delta_aruco_position_frd.point.z + land_height_[workstep]);
				msg.position = {static_cast<float>(tar_x), static_cast<float>(tar_y), static_cast<float>(tar_z)};
				msg.yaw = local_position_yaw + param_yaw_ * aruco_position_frd_yaw;
			}
			else
			{
				msg.position = {static_cast<float>(tar_x), static_cast<float>(tar_y), static_cast<float>(tar_z)};
			}
			if (std::abs(delta_aruco_position_frd.point.x) < allow_visual_err_xyz_ &&
				std::abs(delta_aruco_position_frd.point.y) < allow_visual_err_xyz_ &&
				std::abs(delta_aruco_position_frd.point.z + land_height_[workstep]) < allow_visual_err_xyz_ &&
				(std::abs(delta_aruco_frd_yaw) < allow_visual_err_yaw_ or  std::abs(local_position_z) < 0.20)
				)
			{
				counter_++;
				if (counter_ > 10)
				{
				workstep = 3;
				counter_ = 0;
				take_off = false;
				preperation_precise_landing = false;
				visual_precise_landing = false;
				}
			}

			break;
		case 3:
			workstep = 4;
			break;
		default:
			break;
		}
		RCLCPP_INFO(this->get_logger(), "tar_z, aruco_velosity_frd.point.z = [%f, %f]", tar_z, aruco_velosity_frd.point.z);
		RCLCPP_INFO(this->get_logger(), "aruco_velosity_ned = [%f, %f]", aruco_velosity_ned.point.x, aruco_velosity_ned.point.y);
		RCLCPP_INFO(this->get_logger(), "delta_aruco_position_frd = [%f, %f]", delta_aruco_position_frd.point.x, delta_aruco_position_frd.point.y);
	}
	// RCLCPP_INFO(this->get_logger(), "publisher_count of aruco_subscription = %zu", aruco_subscription_->get_publisher_count());
	// RCLCPP_INFO(this->get_logger(), "now,last = [%ld, %ld]", aruco_pose_time_stamp_now, aruco_pose_time_stamp_last);
	msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
	trajectory_setpoint_publisher_->publish(msg);

	if (aruco_pose_time_stamp_last == aruco_pose_time_stamp_now)
	{
		protect_cnt_++;
		if (protect_cnt_ > 10 * loss_of_aruco_protection_time_)
		{
			loss_of_aruco_protection = true;
		}
	}
	else
	{
		protect_cnt_ = 0;
		loss_of_aruco_protection = false;
	}
	RCLCPP_INFO(this->get_logger(), "protect_cnt_ = [%ld]", protect_cnt_);
	aruco_pose_time_stamp_last = aruco_pose_time_stamp_now;

}

/**
 * @brief Publish vehicle commands
 * @param command   Command code (matches VehicleCommand and MAVLink MAV_CMD codes)
 * @param param1    Command parameter 1
 * @param param2    Command parameter 2
 */
void OffboardControl::publish_vehicle_command(uint16_t command, double param1, double param2, double param3)
{
	VehicleCommand msg{};
	msg.param1 = param1;
	msg.param2 = param2;
	msg.param3 = param3;
	msg.command = command;
	msg.target_system = 1;
	msg.target_component = 1;
	msg.source_system = 1;
	msg.source_component = 1;
	msg.from_external = true;
	msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
	vehicle_command_publisher_->publish(msg);
}

int main(int argc, char *argv[])
{
	std::cout << "Starting offboard control node..." << std::endl;
	setvbuf(stdout, NULL, _IONBF, BUFSIZ);
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<OffboardControl>());

	rclcpp::shutdown();
	return 0;
}
