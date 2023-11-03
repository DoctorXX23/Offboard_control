#include <mavsdk/mavsdk.h>
#include <mavsdk/plugins/action/action.h>
#include <mavsdk/plugins/mission/mission.h>
#include <mavsdk/plugins/telemetry/telemetry.h>
#include <mavsdk/plugins/offboard/offboard.h>
#include <mavsdk/plugins/info/info.h>

#include <chrono>
#include <functional>
#include <future>
#include <iostream>
#include <thread>
#include <string>
#include <utility>

#include <stdint.h>

#include <Eigen/Dense>

#include <rclcpp/rclcpp.hpp>

#include "std_srvs/srv/trigger.hpp"
#include "std_srvs/srv/empty.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/twist.hpp"

#include "uars_ros_interface/msg/battery.hpp"
#include "uars_ros_interface/msg/flightmode.hpp"
#include "uars_ros_interface/msg/gpsinfo.hpp"
#include "uars_ros_interface/msg/heading.hpp"
#include "uars_ros_interface/msg/heartbeat.hpp"
#include "uars_ros_interface/msg/position.hpp"
#include "uars_ros_interface/srv/waypoints.hpp"

namespace offboard
{
    class OffboardControl : public rclcpp::Node
    {
    public:
        OffboardControl();
    private:
        std::unique_ptr<mavsdk::Mavsdk> _mavsdk;
        std::unique_ptr<mavsdk::Action> _action;
        std::shared_ptr<mavsdk::Telemetry> _telemetry;
        std::unique_ptr<mavsdk::Offboard> _offboard;

        rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr _velocitySub;
        void cbVelocity(const geometry_msgs::msg::Twist::SharedPtr aMsg);

        rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr _srvTakeOff;
        void cbTakeOff(const std::shared_ptr<std_srvs::srv::Trigger::Request> aRequest, const std::shared_ptr<std_srvs::srv::Trigger::Response> aResponse);

        rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr _srvStartOffboard;
        void cbStartOffboard(const std::shared_ptr<std_srvs::srv::Trigger::Request> aRequest, const std::shared_ptr<std_srvs::srv::Trigger::Response> aResponse);

        void takeOff();
        std::shared_ptr<mavsdk::System> getSystem(mavsdk::Mavsdk& aMavsdk);
    };
}