#include "offboard_control/offboard_control.hpp"

using namespace std::placeholders;

namespace offboard
{
    OffboardControl::OffboardControl() : Node("OffboardControl")
    {
        rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
		auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);

        declare_parameter("connection", "udp://:14541");

        _mavsdk = std::make_unique<mavsdk::Mavsdk>();

        auto connection = get_parameter("connection").as_string();

        mavsdk::ConnectionResult connectionResult = _mavsdk.get()->add_any_connection(connection);

        if(connectionResult != mavsdk::ConnectionResult::Success)
        {
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Connection failed");

            throw std::runtime_error("Connection failed");
        }

        auto system = getSystem(*_mavsdk);

        if(system == nullptr)
        {
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Timed out waiting for system");

            throw std::runtime_error("Timed out waiting for system");
        }

        _action = std::make_unique<mavsdk::Action>(system);
        _telemetry = std::make_shared<mavsdk::Telemetry>(system);
        _offboard = std::make_unique<mavsdk::Offboard>(system);

        _srvTakeOff = this->create_service<std_srvs::srv::Trigger>("offboard_flier/take_off", std::bind(&OffboardControl::cbTakeOff, this, _1, _2));

        _velocitySub = this->create_subscription<geometry_msgs::msg::Twist>("offboard_flier/velocity", 10, std::bind(&OffboardControl::cbVelocity, this, _1));
    }

    void OffboardControl::cbVelocity(const geometry_msgs::msg::Twist::SharedPtr aMsg)
    {
        mavsdk::Offboard::VelocityBodyYawspeed velocity;

        velocity.down_m_s = aMsg->linear.x;
        velocity.yawspeed_deg_s = aMsg->angular.z;

        _offboard.get()->set_velocity_body(velocity);
    }

    void OffboardControl::takeOff()
    {
        const auto arm_result = _action.get()->arm();
        if(arm_result != mavsdk::Action::Result::Success)
        {
            std::cerr << "Arming failed: " << arm_result << '\n';
            throw std::runtime_error("Arming error");
        }

        std::cout << "Armed\n";

        const auto takeoff_result = _action.get()->takeoff();

        if(takeoff_result != mavsdk::Action::Result::Success)
        {
            std::cerr << "Takeoff failed: " << takeoff_result << '\n';
            throw std::runtime_error("Takeoff error");
        }

        auto in_air_promise = std::promise<void>{};
        auto in_air_future = in_air_promise.get_future();

        _telemetry.get()->subscribe_landed_state(
            [this, &in_air_promise](mavsdk::Telemetry::LandedState state) {
                if(state == mavsdk::Telemetry::LandedState::InAir)
                {
                    std::cout << "Taking off has finished\n.";
                    _telemetry.get()->subscribe_landed_state(nullptr);
                    in_air_promise.set_value();
                }
            });

        in_air_future.wait_for(std::chrono::seconds(10));
        if(in_air_future.wait_for(std::chrono::seconds(3)) == std::future_status::timeout)
        {
            std::cerr << "Takeoff timed out.\n";
            throw std::runtime_error("Takeoff timeout error");
        }
    }

    void OffboardControl::cbTakeOff(const std::shared_ptr<std_srvs::srv::Trigger::Request> aRequest, const std::shared_ptr<std_srvs::srv::Trigger::Response> aResponse)
    {
        try
        {
            takeOff();
        }
        catch(const std::exception& e)
        {
            std::cerr << e.what() << '\n';

            aResponse->success = false;
            aResponse->message = "Could not take off...";
        }

        aResponse->success = true;
        aResponse->message = "Flying...";
    }

    std::shared_ptr<mavsdk::System> OffboardControl::getSystem(mavsdk::Mavsdk& aMavsdk)
    {
        std::cout << "Waiting to discover system...\n";
        auto prom = std::promise<std::shared_ptr<mavsdk::System>>{};
        auto fut = prom.get_future();

        // We wait for new systems to be discovered, once we find one that has an
        // autopilot, we decide to use it.
        aMavsdk.subscribe_on_new_system([&aMavsdk, &prom]() {
            auto system = aMavsdk.systems().back();

            if(system->has_autopilot())
            {
                std::cout << "Discovered autopilot\n";

                // Unsubscribe again as we only want to find one system.
                aMavsdk.subscribe_on_new_system(nullptr);
                prom.set_value(system);
            }
        });

        // We usually receive heartbeats at 1Hz, therefore we should find a
        // system after around 3 seconds max, surely.
        if(fut.wait_for(std::chrono::seconds(5)) == std::future_status::timeout)
        {
            std::cerr << "No autopilot found.\n";
            return nullptr;
        }

        // Get discovered system now.
        return fut.get();
    }
}

int main(int argc, char *argv[])
{
    std::cout << "Starting OffboardFlier node..." << std::endl;

    setvbuf(stdout, NULL, _IONBF, BUFSIZ);

    rclcpp::init(argc, argv);

    try
    {
        rclcpp::spin(std::make_shared<offboard::OffboardControl>());
    }
    catch(const std::runtime_error& e)
    {
        std::cout << "Exception: " << e.what() << std::endl;
    }
    catch(...)
    {

    }

    rclcpp::shutdown();
    
    return 0;
}