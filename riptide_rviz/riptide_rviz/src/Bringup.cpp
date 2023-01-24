#include "riptide_rviz/Bringup.hpp"
#include <iostream>
#include <filesystem>
#include <chrono>
#include <string>
#include <rviz_common/display_context.hpp>

using namespace std::chrono_literals;

namespace riptide_rviz
{
    Bringup::Bringup(QWidget *parent) : rviz_common::Panel(parent)
    {
        setFocusPolicy(Qt::ClickFocus);

        uiPanel = new Ui_Bringup();
        uiPanel->setupUi(this);

        auto options = rclcpp::NodeOptions().arguments({});
        clientNode = std::make_shared<rclcpp::Node>("riptide_rviz_bringup", options);
    }

    void Bringup::onInitialize()
    {
        // refresh UI elements so they start displayed correctly
        bringupListRefresh();

        // create a spin timer
        spinTimer = new QTimer(this);
        connect(spinTimer, &QTimer::timeout, [this](void)
                { rclcpp::spin_some(clientNode); });
        spinTimer->start(50);

        // Connect UI signals for bringup
        // create the bringup timer
        bringupCheckTimer = new QTimer(this);
        connect(bringupCheckTimer, &QTimer::timeout, [this](void)
                { checkBringupStatus(); });
        connect(uiPanel->bringupRefresh, &QPushButton::clicked, [this](void)
                { bringupListRefresh(); });
        connect(uiPanel->bringupStart, &QPushButton::clicked, [this](void)
                { startBringup(); });
        connect(uiPanel->bringupHost, SIGNAL(currentIndexChanged(int)), SLOT(handleBringupHost(int)));
        connect(uiPanel->bringupStop, &QPushButton::clicked, [this](void)
                { stopBringup(); });
    }

    void Bringup::load(const rviz_common::Config &config)
    {
        rviz_common::Panel::load(config);
    }

    void Bringup::save(rviz_common::Config config) const
    {
        rviz_common::Panel::save(config);
    }

    bool Bringup::event(QEvent *event)
    {
    }

    Bringup::~Bringup()
    {
        // master window control removal
        delete uiPanel;

        // remove the timers
        delete spinTimer, bringupCheckTimer;
    }

    void Bringup::bringupListRefresh()
    {
        // get the list of nodes available
        std::vector<std::string> names = clientNode->get_node_names();

        // clear the entries in the combo boxes
        uiPanel->bringupHost->clear();

        // filter names down to launch nodes
        std::vector<std::string> launchNodes;
        auto filt = [](const auto &s)
        { return s.find("_launch_manager") != std::string::npos; };
        std::copy_if(names.begin(), names.end(), std::back_inserter(launchNodes), filt);

        // make sure we have availiable nodes in the list, otherwise place a blank list entry
        if (launchNodes.size() > 0)
        {
            uiPanel->bringupHost->addItem("None Selected");
            for (std::string const &name : launchNodes)
            {
                // push these into the combo box
                uiPanel->bringupHost->addItem(QString::fromStdString(name.substr(0, name.find_last_of('l') - 1)));
            }
            uiPanel->bringupStart->setDisabled(false);
        }
        else
        {
            uiPanel->bringupHost->addItem("None");
            uiPanel->bringupStart->setDisabled(true);
        }

        // clear the list of bringup files
        uiPanel->bringupFile->clear();
        uiPanel->bringupFile->addItem("None Selected");

        // populate with new files
        std::string bringupFilesDir = ament_index_cpp::get_package_share_directory(BRINGUP_PKG) + "/launch";
        for (const auto &entry : std::filesystem::directory_iterator(bringupFilesDir))
        {
            std::string file = std::string(entry.path());
            if (file.find(".launch.py") != std::string::npos)
            {
                uiPanel->bringupFile->addItem(QString::fromStdString(file.substr(file.find_last_of('/') + 1)));
            }
        }
    }

    void Bringup::handleBringupHost(int selection)
    {
        std::string targetNode = uiPanel->bringupHost->itemText(selection).toStdString();

        // make sure the event wasnt generated by us
        if (targetNode != "None Selected")
        {
            // overwrite the old clients
            bringupStartClient = clientNode->create_client<launch_msgs::srv::StartLaunch>(targetNode + "/start_launch");
            bringupListClient = clientNode->create_client<launch_msgs::srv::ListLaunch>(targetNode + "/list_launch");
            bringupStopClient = clientNode->create_client<launch_msgs::srv::StopLaunch>(targetNode + "/stop_launch");
        }
    }

    void Bringup::startBringup()
    {
        // make sure that the bringup file is selected
        std::string targetFile = uiPanel->bringupFile->currentText().toStdString();

        // validate selection
        if (targetFile != "None" && targetFile != "None Selected")
        {
            // disable the start button and enable the stop button
            uiPanel->bringupStart->setDisabled(true);
            uiPanel->bringupStop->setDisabled(false);
            uiPanel->bringupRefresh->setDisabled(true);

            launch_msgs::srv::StartLaunch::Request::SharedPtr startReq = std::make_shared<launch_msgs::srv::StartLaunch::Request>();
            startReq->launch_file = targetFile;
            startReq->package = BRINGUP_PKG;

            // check client not busy
            while (!bringupStartClient->wait_for_service(100ms))
                if (!rclcpp::ok())
                    return;

            // make the request and wait for the future to be valid
            auto bringupFuture = bringupStartClient->async_send_request(startReq);
            if (rclcpp::spin_until_future_complete(clientNode, bringupFuture, 1s) == rclcpp::FutureReturnCode::SUCCESS)
            {

                auto result = bringupFuture.get();
                if (result->started)
                {
                    // record the bringup id
                    bringupID = result->formed_launch.launch_id;

                    // start the check timer to make sure the service is still alive
                    bringupCheckTimer->start(BRINGUP_POLLING_RATE);

                    // bail early if everything went okay
                    return;
                }
            }

            bringupID = -1;

            // reset the buttons durig an error
            uiPanel->bringupStart->setDisabled(false);
            uiPanel->bringupStop->setDisabled(true);
            uiPanel->bringupRefresh->setDisabled(false);
        }
    }

    void Bringup::checkBringupStatus()
    {
        launch_msgs::srv::ListLaunch::Request::SharedPtr listReq = std::make_shared<launch_msgs::srv::ListLaunch::Request>();
        listReq->status = launch_msgs::msg::LaunchID::UNKNOWN;

        auto start = clientNode->get_clock()->now();
        while (!bringupListClient->wait_for_service(100ms))
            if (!rclcpp::ok() || clientNode->get_clock()->now() - start > 1s)
                return;

        auto future = bringupListClient->async_send_request(listReq);
        if (rclcpp::spin_until_future_complete(clientNode, future, 1s) == rclcpp::FutureReturnCode::SUCCESS)
        {
            // find the launch ID we want
            auto launches = future.get()->launches;
            for (auto launch : launches)
            {
                // we found the one we want and it has died
                if (launch.launch_id == bringupID && launch.status != launch.RUNNING)
                {
                    uiPanel->bringupStart->setDisabled(false);
                    uiPanel->bringupStop->setDisabled(true);
                    uiPanel->bringupRefresh->setDisabled(false);

                    bringupCheckTimer->stop();
                }
            }
        }
    }

    void Bringup::stopBringup()
    {
        launch_msgs::srv::StopLaunch::Request::SharedPtr stopReq = std::make_shared<launch_msgs::srv::StopLaunch::Request>();
        stopReq->launch_id = bringupID;

        while (!bringupStopClient->wait_for_service(100ms))
            if (!rclcpp::ok())
                return;

        auto future = bringupStopClient->async_send_request(stopReq);
        if (rclcpp::spin_until_future_complete(clientNode, future, 1s) == rclcpp::FutureReturnCode::SUCCESS)
        {
            if (future.get()->stopped)
            {
                // launch has died, reset buttons
                uiPanel->bringupStart->setDisabled(false);
                uiPanel->bringupStop->setDisabled(true);
                uiPanel->bringupRefresh->setDisabled(false);

                bringupCheckTimer->stop();
            }
        }
    }

} // namespace riptide_rviz

#include <pluginlib/class_list_macros.hpp> // NOLINT
PLUGINLIB_EXPORT_CLASS(riptide_rviz::Bringup, rviz_common::Panel);