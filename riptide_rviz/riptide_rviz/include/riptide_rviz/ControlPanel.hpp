#pragma once
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <riptide_msgs2/msg/controller_command.hpp>
#include <riptide_msgs2/msg/kill_switch_report.hpp>
#include <riptide_msgs2/action/follow_path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/bool.hpp>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <ament_index_cpp/get_package_prefix.hpp>
#include <rviz_common/panel.hpp>
#include <rviz_common/config.hpp>

#include "ui_ControlPanel.h"
#include <QTimer>

namespace riptide_rviz
{
    using FollowPath = riptide_msgs2::action::FollowPath;
    using GHFollowPath = rclcpp_action::ClientGoalHandle<riptide_msgs2::action::FollowPath>;

    class ControlPanel : public rviz_common::Panel
    {
        Q_OBJECT public : ControlPanel(QWidget *parent = 0);
        ~ControlPanel();

        void load(const rviz_common::Config &config) override;
        void save(rviz_common::Config config) const override;

        void onInitialize() override;

        // ROS Subscriber callbacks
        void odomCallback(const nav_msgs::msg::Odometry &msg);
        void steadyCallback(const std_msgs::msg::Bool & msg);
        void selectedPose(const geometry_msgs::msg::PoseStamped & msg);

        // ROS timer callbacks
        void sendKillMsgTimer();

    protected Q_SLOTS:
        // QT slots (function callbacks)
        // slots for handling mode setting of the controller
        void handleEnable();
        void handleDisable(); // pressing disable asserts kill and clears command
        void switchMode(uint8_t mode, bool override=false);

        // slots for controlling the UI
        void toggleDegrees();
        void refreshUI();

        // slots for sending commands to the vehicle
        void handleLocalDive();
        void handleCurrent();
        void handleCommand();

    protected:
        bool event(QEvent *event);

    private:
        // UI Panel instance
        Ui_ControlPanel *uiPanel;

        // robot namespace used for config save and load
        std::string robot_ns;

        // doubles for max depth and duration for odom timeout
        double max_depth_in_place, tgt_in_place_depth;
        std::chrono::duration<double> odom_timeout;

        // Map frame ID used in FollowPath
        std::string map_frame_id;

        // mode for sending commands to the controller
        uint8_t ctrlMode;

        // last time we have recieved odom
        builtin_interfaces::msg::Time odomTime;

        // internal flags
        bool vehicleEnabled = false;
        bool degreeReadout = true;

        // core stuff for creating and managing a ROS node
        rclcpp::Node::SharedPtr clientNode;
        QTimer *spinTimer;

        // QT ui timer for handling data freshness
        QTimer *uiTimer;

        // publishers
        rclcpp::Publisher<riptide_msgs2::msg::ControllerCommand>::SharedPtr ctrlCmdLinPub, ctrlCmdAngPub;
        rclcpp::Publisher<riptide_msgs2::msg::KillSwitchReport>::SharedPtr killStatePub;

        // ROS Timers
        rclcpp::TimerBase::SharedPtr killPubTimer;

        // ROS Subscribers
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odomSub;
        rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr steadySub;
        rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr selectPoseSub;

        // ROS Action Clients
        rclcpp_action::Client<riptide_msgs2::action::FollowPath>::SharedPtr followPathClient;
        
        // Action Client Callback
        void followPathGoalResponseCb(const GHFollowPath::SharedPtr & goal_handle);
        void followPathFeedbackCb(GHFollowPath::SharedPtr, const std::shared_ptr<const FollowPath::Feedback> feedback);
        void followPathResultCb(const GHFollowPath::WrappedResult & result);

        // Helper functons
        geometry_msgs::msg::PoseStamped getPoseStamped(double x, double y, double z, double roll, double pitch, double yaw);
    };

} // namespace riptide_rviz