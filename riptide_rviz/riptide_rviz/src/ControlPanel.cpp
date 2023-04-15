#include "riptide_rviz/ControlPanel.hpp"

#include <QMessageBox>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <chrono>
#include <algorithm>
#include <iostream>
#include <functional>

#include <rviz_common/logging.hpp>

using namespace std::chrono_literals;
using std::placeholders::_1;
using std::placeholders::_2;

namespace riptide_rviz
{
    ControlPanel::ControlPanel(QWidget *parent) : rviz_common::Panel(parent)
    {
        setFocusPolicy(Qt::ClickFocus);

        uiPanel = new Ui_ControlPanel();
        uiPanel->setupUi(this);

        // create the RCLCPP client
        auto options = rclcpp::NodeOptions().arguments({});
        clientNode = std::make_shared<rclcpp::Node>("riptide_rviz_control", options);

        // create the default message
        ctrlMode = riptide_msgs2::msg::ControllerCommand::DISABLED;

        RVIZ_COMMON_LOG_INFO("Constructed control panel");
    }

    void ControlPanel::onInitialize()
    {
        // RVIZ_COMMON_LOG_INFO("Initializing");
        // create a spin timer
        spinTimer = new QTimer(this);
        connect(spinTimer, &QTimer::timeout, [this](void)
                { rclcpp::spin_some(clientNode); });
        spinTimer->start(50);

        // Connect UI signals for controlling the riptide vehicle
        connect(uiPanel->ctrlEnable, &QPushButton::clicked, [this](void)
                { handleEnable(); });
        connect(uiPanel->ctrlDisable, &QPushButton::clicked, [this](void)
                { handleDisable(); });
        connect(uiPanel->ctrlDegOrRad, &QPushButton::clicked, [this](void)
                { toggleDegrees(); });

        // mode seting buttons
        connect(uiPanel->ctrlModePos, &QPushButton::clicked,
                [this](void)
                { switchMode(riptide_msgs2::msg::ControllerCommand::POSITION); });
        connect(uiPanel->ctrlModeVel, &QPushButton::clicked,
                [this](void)
                { switchMode(riptide_msgs2::msg::ControllerCommand::VELOCITY); });
        connect(uiPanel->ctrlModeFFD, &QPushButton::clicked,
                [this](void)
                { switchMode(riptide_msgs2::msg::ControllerCommand::FEEDFORWARD); });
        connect(uiPanel->ctrlModeVel, &QPushButton::clicked,
                [this](void)
                { switchMode(22); });

        // command sending buttons
        connect(uiPanel->ctrlDiveInPlace, &QPushButton::clicked, [this](void)
                { handleLocalDive(); });
        connect(uiPanel->ctrlFwdCurrent, &QPushButton::clicked, [this](void)
                { handleCurrent(); });
        connect(uiPanel->CtrlSendCmd, &QPushButton::clicked, [this](void)
                { handleCommand(); });

        uiPanel->pathProgressBar->setMaximum(100);
        uiPanel->pathProgressBar->setFormat("%v%");

        RVIZ_COMMON_LOG_INFO("Initialized control panel");
    }

    void ControlPanel::load(const rviz_common::Config &config)
    {
        // load the parent class config
        rviz_common::Panel::load(config);

        RVIZ_COMMON_LOG_INFO("Loaded parent panel config");

        // create our value containers
        QString * str = new QString();
        float * configVal = new float();

        // load the namesapce param
        if(config.mapGetString("robot_namespace", str)){
            robot_ns = str->toStdString();
        } else {
            // default value
            robot_ns = "/talos";
            RVIZ_COMMON_LOG_WARNING("Loading default value for 'namespace'");
        }

        if (config.mapGetString("map_frame_id", str)) {
            map_frame_id = str->toStdString();
        } else {
            // default
            map_frame_id = "map";
        }

        if(config.mapGetFloat("odom_timeout", configVal)){
            odom_timeout = std::chrono::duration<double>(*configVal);
        } else {
            // default value
            odom_timeout = std::chrono::duration<double>(2.5);
            RVIZ_COMMON_LOG_WARNING("Loading default value for 'odom_timeout'");
        }

        if(config.mapGetFloat("tgt_in_place_depth", configVal)){
            tgt_in_place_depth = *configVal;
        } else {
            // default value
            tgt_in_place_depth = -0.75;
            RVIZ_COMMON_LOG_WARNING("Loading default value for 'tgt_in_place_depth'");
        }

        if(config.mapGetFloat("max_depth_in_place", configVal)){
            max_depth_in_place = *configVal;
        } else {
            // default value
            max_depth_in_place = -0.5;
            RVIZ_COMMON_LOG_WARNING("Loading default value for 'max_depth_in_place'");
        }

        // Free the allocated containers
        delete str;
        delete configVal; 

        // create the timer but hold on starting it as things may not have been fully initialized yet
        uiTimer = new QTimer(this);
        connect(uiTimer, &QTimer::timeout, [this](void)
                { refreshUI(); });

        // setup goal_pose sub
        selectPoseSub = clientNode->create_subscription<geometry_msgs::msg::PoseStamped>(
            "goal_pose", rclcpp::SystemDefaultsQoS(),
            std::bind(&ControlPanel::selectedPose, this, _1)); 

        // setup the ROS topics that depend on namespace
        // make publishers
        killStatePub = clientNode->create_publisher<riptide_msgs2::msg::KillSwitchReport>(robot_ns + "/control/software_kill", rclcpp::SystemDefaultsQoS());
        ctrlCmdLinPub = clientNode->create_publisher<riptide_msgs2::msg::ControllerCommand>(robot_ns + "/controller/linear", rclcpp::SystemDefaultsQoS());
        ctrlCmdAngPub = clientNode->create_publisher<riptide_msgs2::msg::ControllerCommand>(robot_ns + "/controller/angular", rclcpp::SystemDefaultsQoS());

        // make ROS Subscribers
        odomSub = clientNode->create_subscription<nav_msgs::msg::Odometry>(
            robot_ns + "/odometry/filtered", rclcpp::SystemDefaultsQoS(),
            std::bind(&ControlPanel::odomCallback, this, _1));
        steadySub = clientNode->create_subscription<std_msgs::msg::Bool>(
            robot_ns + "/controller/steady", rclcpp::SystemDefaultsQoS(),
            std::bind(&ControlPanel::steadyCallback, this, _1));  

        followPathClient = rclcpp_action::create_client<FollowPath>(clientNode, robot_ns + "/follow_path");

        // now we can start the UI refresh timer
        uiTimer->start(100);
        
        // and start the kill switch pub timer
        killPubTimer = clientNode->create_wall_timer(50ms, std::bind(&ControlPanel::sendKillMsgTimer, this));

        RVIZ_COMMON_LOG_INFO("Loading config complete");
    }

    void ControlPanel::save(rviz_common::Config config) const
    {
        rviz_common::Panel::save(config);

        // write our config values
        config.mapSetValue("robot_namespace", QString::fromStdString(robot_ns));
        config.mapSetValue("odom_timeout", odom_timeout.count());
        config.mapSetValue("max_depth_in_place", max_depth_in_place);
        config.mapSetValue("tgt_in_place_depth", tgt_in_place_depth);
        config.mapSetValue("map_frame_id", QString::fromStdString(map_frame_id));
    }

    bool ControlPanel::event(QEvent *event)
    {
    }

    ControlPanel::~ControlPanel()
    {
        // master window control removal
        delete uiPanel;

        // remove the timers
        delete spinTimer, uiTimer;

        rclcpp::shutdown();
    }

    // slots for handling mode setting of the controller
    void ControlPanel::handleEnable()
    {
        vehicleEnabled = true;
        uiPanel->ctrlEnable->setEnabled(false);
        uiPanel->ctrlDisable->setEnabled(true);
    }

    void ControlPanel::handleDisable()
    {
        vehicleEnabled = false;
        uiPanel->ctrlEnable->setEnabled(true);
        uiPanel->ctrlDisable->setEnabled(false);

        // clear the controller command mode
        switchMode(riptide_msgs2::msg::ControllerCommand::DISABLED, true);
    }

    void ControlPanel::switchMode(uint8_t mode, bool override)
    {
        // check the vehicle is enabled or we are overriding
        if(vehicleEnabled || override){
            ctrlMode = mode;
            switch (ctrlMode)
            {
            case riptide_msgs2::msg::ControllerCommand::POSITION:
                uiPanel->ctrlModeFFD->setEnabled(true);
                uiPanel->ctrlModeVel->setEnabled(true);
                uiPanel->ctrlModePos->setEnabled(false);
                uiPanel->ctrlModeTele->setEnabled(true);

                break;
            case riptide_msgs2::msg::ControllerCommand::VELOCITY:
                uiPanel->ctrlModeFFD->setEnabled(true);
                uiPanel->ctrlModeVel->setEnabled(false);
                uiPanel->ctrlModePos->setEnabled(true);
                uiPanel->ctrlModeTele->setEnabled(true);

                break;
            case riptide_msgs2::msg::ControllerCommand::FEEDFORWARD:
                uiPanel->ctrlModeFFD->setEnabled(false);
                uiPanel->ctrlModeVel->setEnabled(true);
                uiPanel->ctrlModePos->setEnabled(true);
                uiPanel->ctrlModeTele->setEnabled(true);
                break;
            case riptide_msgs2::msg::ControllerCommand::DISABLED:
                uiPanel->ctrlModeFFD->setEnabled(true);
                uiPanel->ctrlModeVel->setEnabled(true);
                uiPanel->ctrlModePos->setEnabled(true);
                uiPanel->ctrlModeTele->setEnabled(true);
                break;
            default:
                RVIZ_COMMON_LOG_ERROR("Button not yet operable");
                break;
            }
        }
    }

    void ControlPanel::refreshUI()
    {
        // handle timing out the UI buttons if odom gets too stale
        auto diff = clientNode->get_clock()->now() - odomTime;
        if (diff.to_chrono<std::chrono::seconds>() > odom_timeout || !vehicleEnabled)
        {

            // the odom has timed out
            if (uiPanel->CtrlSendCmd->isEnabled()){
                RVIZ_COMMON_LOG_WARNING("Odom timed out, or vehicle disabled! disabling local control buttons");

                // disable the vehicle
                handleDisable();
            }

            // also disable command sending
            if (!vehicleEnabled) {
                RVIZ_COMMON_LOG_WARNING("vehicleEnabled == false: Disabling ctrlSendCmd because of odom_timeout");
            }

            if (diff.to_chrono<std::chrono::seconds>() > odom_timeout) {
                RVIZ_COMMON_LOG_WARNING("Odom Timeout reached: Disabling ctrlSendCmd because of odom_timeout");
            }

            uiPanel->ctrlDiveInPlace->setEnabled(false);
            uiPanel->CtrlSendCmd->setEnabled(false);
        }
        else
        {
            uiPanel->CtrlSendCmd->setEnabled(true);

            // check the current depth. if we are below 0.5m, disable the submerge in place button
            bool convOk;
            double z = uiPanel->cmdCurrZ->text().toDouble(&convOk);
            if (convOk && z < max_depth_in_place)
            {
                uiPanel->ctrlDiveInPlace->setEnabled(false);
            }
            else
            {
                uiPanel->ctrlDiveInPlace->setEnabled(true);
            }
        }
    }

    // slots for sending commands to the vehicle
    void ControlPanel::handleLocalDive()
    {
        // first take the current readout and hold it
        // only need xy and yaw, we discard roll and pitch and z
        double x, y, yaw;
        bool convOk[3];

        // make sure that the conversion goes okay as well
        x = uiPanel->cmdCurrX->text().toDouble(&convOk[0]);
        y = uiPanel->cmdCurrY->text().toDouble(&convOk[1]);
        yaw = uiPanel->cmdCurrX->text().toDouble(&convOk[2]);

        if (std::any_of(std::begin(convOk), std::end(convOk), [](bool i)
                        { return !i; }))
        {
            RVIZ_COMMON_LOG_ERROR("Failed to convert current position to floating point");
            
            // set the red stylesheet
            uiPanel->ctrlDiveInPlace->setStyleSheet("QPushButton{color:black; background: red;}");

            // create a timer to clear it in 1 second
            QTimer::singleShot(1000, [this](void)
                               { uiPanel->ctrlDiveInPlace->setStyleSheet(""); });
            return;
        }

        // build the linear control message
        // auto override the control mode to position 
        auto linCmd = riptide_msgs2::msg::ControllerCommand();
        linCmd.setpoint_vect.x = x;
        linCmd.setpoint_vect.y = y;
        // automatically go to configured depth below surface
        linCmd.setpoint_vect.z = tgt_in_place_depth; 
        linCmd.mode = riptide_msgs2::msg::ControllerCommand::POSITION;

        // check the angle mode button
        if(degreeReadout){
            yaw *= M_PI / 180.0; 
        }

        // convert RPY to quaternion
        tf2::Quaternion quat;
        quat.setRPY(0, 0, yaw);

        // build the angular message
        auto angular = tf2::toMsg(quat);
        auto angCmd = riptide_msgs2::msg::ControllerCommand();
        angCmd.setpoint_quat = angular;
        angCmd.mode = riptide_msgs2::msg::ControllerCommand::POSITION;

        // send the control messages
        ctrlCmdLinPub->publish(linCmd);
        ctrlCmdAngPub->publish(angCmd);
    }

    void ControlPanel::toggleDegrees()
    {
        degreeReadout = !degreeReadout;
        if (degreeReadout)
        {
            uiPanel->ctrlDegOrRad->setText("Degrees");
        }
        else
        {
            uiPanel->ctrlDegOrRad->setText("Radians");
        }
    }

    void ControlPanel::handleCurrent()
    {
        // take the values from the readouts
        QString x, y, z, roll, pitch, yaw;
        x = uiPanel->cmdCurrX->text();
        y = uiPanel->cmdCurrY->text();
        z = uiPanel->cmdCurrZ->text();
        roll = uiPanel->cmdCurrR->text();
        pitch = uiPanel->cmdCurrP->text();
        yaw = uiPanel->cmdCurrYaw->text();

        // take the values and propagate them into the requested values
        uiPanel->cmdReqX->setText(x);
        uiPanel->cmdReqY->setText(y);
        uiPanel->cmdReqZ->setText(z);
        uiPanel->cmdReqR->setText(roll);
        uiPanel->cmdReqP->setText(pitch);
        uiPanel->cmdReqYaw->setText(yaw);
    }

    void ControlPanel::handleCommand()
    {
        // first take the current readout and hold it
        // only need xy and yaw, we discard roll and pitch and z
        double xCurrent, yCurrent, zCurrent, rollCurrent, pitchCurrent, yawCurrent;
        double xEnd, yEnd, zEnd, rollEnd, pitchEnd, yawEnd;
        bool convOkCurrent[6];
        bool convOkEnd[6];

        uiPanel->pathProgressBar->setValue(0);
        uiPanel->pathProgressBar->setStyleSheet("");
        uiPanel->pathProgressBar->setFormat("%v%");

        // make sure that the conversion goes okay as well
        xCurrent = uiPanel->cmdReqX->text().toDouble(&convOkCurrent[0]);
        yCurrent = uiPanel->cmdReqY->text().toDouble(&convOkCurrent[1]);
        zCurrent = uiPanel->cmdReqZ->text().toDouble(&convOkCurrent[2]);
        rollCurrent = uiPanel->cmdReqR->text().toDouble(&convOkCurrent[3]);
        pitchCurrent = uiPanel->cmdReqP->text().toDouble(&convOkCurrent[4]);
        yawCurrent = uiPanel->cmdReqYaw->text().toDouble(&convOkCurrent[5]);

        xEnd = uiPanel->cmdReqX->text().toDouble(&convOkEnd[0]);
        yEnd = uiPanel->cmdReqY->text().toDouble(&convOkEnd[1]);
        zEnd = uiPanel->cmdReqZ->text().toDouble(&convOkEnd[2]);
        rollEnd = uiPanel->cmdReqR->text().toDouble(&convOkEnd[3]);
        pitchEnd = uiPanel->cmdReqP->text().toDouble(&convOkEnd[4]);
        yawEnd = uiPanel->cmdReqYaw->text().toDouble(&convOkEnd[5]);

        bool currentConversionFailed = std::any_of(std::begin(convOkCurrent), std::end(convOkCurrent), [](bool i)
                        { return !i; });
        bool endConversionFailed = std::any_of(std::begin(convOkEnd), std::end(convOkEnd), [](bool i)
                        { return !i; });

        if (currentConversionFailed || endConversionFailed)
        {
            if (currentConversionFailed) 
            {
                RVIZ_COMMON_LOG_ERROR("Failed to convert current position to floating point");
            }

            if (endConversionFailed) {
                RVIZ_COMMON_LOG_ERROR("Failed to convert target position to floating point");
            }

            // set the red stylesheet
            uiPanel->CtrlSendCmd->setStyleSheet("QPushButton{color:black; background: red;}");

            // create a timer to clear it in 1 second
            QTimer::singleShot(1000, [this](void)
                               { uiPanel->CtrlSendCmd->setStyleSheet(""); });
            return;
        }

        // Check the distance between the start and end point
        double distance = std::sqrt(std::pow(xEnd - xCurrent, 2) 
                                + std::pow(yEnd - yCurrent, 2) 
                                + std::pow(zEnd - zCurrent, 2));
        
        if (distance < 1.0) {
            // Small distances should send a regular ControllerCommand message

            // now we can build the command and send it
            // build the linear control message
            auto linCmd = riptide_msgs2::msg::ControllerCommand();
            linCmd.setpoint_vect.x = xCurrent;
            linCmd.setpoint_vect.y = yCurrent;
            linCmd.setpoint_vect.z = zCurrent;
            linCmd.mode = ctrlMode;

            // if we are in position, we use quat, otherwise use the vector
            auto angCmd = riptide_msgs2::msg::ControllerCommand();
            angCmd.mode = ctrlMode;

            if (ctrlMode == riptide_msgs2::msg::ControllerCommand::POSITION)
            {
                // check the angle mode button
                if(degreeReadout){
                    rollCurrent *= M_PI / 180.0; 
                    pitchCurrent *= M_PI / 180.0; 
                    yawCurrent *= M_PI / 180.0; 
                }

                // convert RPY to quaternion
                tf2::Quaternion quat;
                quat.setRPY(rollCurrent, pitchCurrent, yawCurrent);

                // build the angular quat for message
                angCmd.setpoint_quat = tf2::toMsg(quat);
            } else {
                // build the vector
                angCmd.setpoint_vect.x = rollCurrent;
                angCmd.setpoint_vect.y = pitchCurrent;
                angCmd.setpoint_vect.z = yawCurrent;
            }

            // send the control messages
            ctrlCmdLinPub->publish(linCmd);
            ctrlCmdAngPub->publish(angCmd);
        } else {
            if (!followPathClient->action_server_is_ready()) {
                QMessageBox::critical(this, "Error Sending Path", "Path action server is unavailable");
                return;
            }
            // Large distances should send a path to the action server
            
            auto start = getPoseStamped(xCurrent,
                                        yCurrent,
                                        zCurrent,
                                        rollCurrent,
                                        pitchCurrent,
                                        yawCurrent);

            auto end = getPoseStamped(xEnd,
                                      yEnd,
                                      zEnd,
                                      rollEnd,
                                      pitchEnd,
                                      yawEnd);

            auto goal_msg = FollowPath::Goal();
            goal_msg.path_points = {start, end};
            

            auto send_goal_options = rclcpp_action::Client<FollowPath>::SendGoalOptions();
            send_goal_options.goal_response_callback = std::bind(&ControlPanel::followPathGoalResponseCb, this, _1);
            send_goal_options.feedback_callback = std::bind(&ControlPanel::followPathFeedbackCb, this, _1, _2);
            send_goal_options.result_callback = std::bind(&ControlPanel::followPathResultCb, this, _1);
            followPathClient->async_send_goal(goal_msg, send_goal_options);
        }
    }

    geometry_msgs::msg::PoseStamped ControlPanel::getPoseStamped(double x, double y, double z, double roll, double pitch, double yaw) {
        geometry_msgs::msg::PoseStamped pose;
        pose.header.frame_id = map_frame_id;

        pose.pose.position.x = x;
        pose.pose.position.y = y;
        pose.pose.position.z = z;

        // check the angle mode button
        if(degreeReadout) {
            roll *= M_PI / 180.0; 
            pitch *= M_PI / 180.0; 
            yaw *= M_PI / 180.0; 
        }

        // convert RPY to quaternion
        tf2::Quaternion quat;
        quat.setRPY(roll, pitch, yaw);

        // build the angular quat for message
        pose.pose.orientation = tf2::toMsg(quat);

        return pose;
    }

    void ControlPanel::followPathGoalResponseCb(const GHFollowPath::SharedPtr & goal_handle) {
        if (!goal_handle) {
            QMessageBox::critical(this, "Error Sending Path", "Path action server rejected the action");
            RVIZ_COMMON_LOG_ERROR("followPathGoalResponseCb: Server has rejected the goal.");
        }
    }

    void ControlPanel::followPathFeedbackCb(GHFollowPath::SharedPtr, const std::shared_ptr<const FollowPath::Feedback> feedback) {
        uiPanel->pathProgressBar->setValue((feedback->current_prog / feedback->expected_prog) * 100);
    }

    void ControlPanel::followPathResultCb(const GHFollowPath::WrappedResult & result) {
        if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
            uiPanel->pathProgressBar->setValue(100);
            uiPanel->pathProgressBar->setFormat("SUCCEEDED");
        } else {
            uiPanel->pathProgressBar->setStyleSheet("QProgressBar {color: white; background: rgb(140, 100, 100);}");
            uiPanel->pathProgressBar->setFormat("ERROR");
        }
    }


    void ControlPanel::odomCallback(const nav_msgs::msg::Odometry &msg)
    {
        // parse the quaternion to RPY
        tf2::Quaternion q(msg.pose.pose.orientation.x, msg.pose.pose.orientation.y,
                          msg.pose.pose.orientation.z, msg.pose.pose.orientation.w);
        tf2::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);

        // save the header timestamp
        odomTime = msg.header.stamp;

        // convert to degrees if its what we're showing
        if (degreeReadout)
        {
            roll *= 180.0 / M_PI;
            pitch *= 180.0 / M_PI;
            yaw *= 180.0 / M_PI;
        }

        // show it to the user
        uiPanel->cmdCurrR->setText(QString::number(roll, 'f', 2));
        uiPanel->cmdCurrP->setText(QString::number(pitch, 'f', 2));
        uiPanel->cmdCurrYaw->setText(QString::number(yaw, 'f', 2));

        uiPanel->cmdCurrX->setText(QString::number(msg.pose.pose.position.x, 'f', 2));
        uiPanel->cmdCurrY->setText(QString::number(msg.pose.pose.position.y, 'f', 2));
        uiPanel->cmdCurrZ->setText(QString::number(msg.pose.pose.position.z, 'f', 2));
    }

    void ControlPanel::selectedPose(const geometry_msgs::msg::PoseStamped & msg){
        // check position control mode !!!
        if (ctrlMode == riptide_msgs2::msg::ControllerCommand::POSITION){
            // use z depth from odom
            auto z = uiPanel->cmdCurrZ->text();
            // update the values set by the selected pose
            // take the values and propagate them into the requested values
            uiPanel->cmdReqZ->setText(z);
            uiPanel->cmdReqX->setText(QString::number(msg.pose.position.x, 'f', 2));
            uiPanel->cmdReqY->setText(QString::number(msg.pose.position.y, 'f', 2));

            // convert yaw only quat to rpy and populate
            tf2::Quaternion q(msg.pose.orientation.x, msg.pose.orientation.y,
                            msg.pose.orientation.z, msg.pose.orientation.w);
            tf2::Matrix3x3 m(q);
            double roll, pitch, yaw;
            m.getRPY(roll, pitch, yaw);

            // convert to degrees if its what we're showing
            if (degreeReadout)
            {
                roll *= 180.0 / M_PI;
                pitch *= 180.0 / M_PI;
                yaw *= 180.0 / M_PI;
            }

            uiPanel->cmdReqR->setText(QString::number(roll, 'f', 2));
            uiPanel->cmdReqP->setText(QString::number(pitch, 'f', 2));
            uiPanel->cmdReqYaw->setText(QString::number(yaw, 'f', 2));
        }
    }

    void ControlPanel::steadyCallback(const std_msgs::msg::Bool &msg)
    {
        uiPanel->cmdSteady->setEnabled(msg.data);
    }

    // ROS timer callbacks
    void ControlPanel::sendKillMsgTimer()
    {
        auto killMsg = riptide_msgs2::msg::KillSwitchReport();
        killMsg.kill_switch_id = riptide_msgs2::msg::KillSwitchReport::KILL_SWITCH_RQT_CONTROLLER;
        killMsg.sender_id = "/riptide_rviz_control";
        killMsg.switch_asserting_kill = !vehicleEnabled;
        killMsg.switch_needs_update = uiPanel->ctrlRequireKill->isChecked();

        killStatePub->publish(killMsg);
    }

} // namespace riptide_rviz

#include <pluginlib/class_list_macros.hpp> // NOLINT
PLUGINLIB_EXPORT_CLASS(riptide_rviz::ControlPanel, rviz_common::Panel);