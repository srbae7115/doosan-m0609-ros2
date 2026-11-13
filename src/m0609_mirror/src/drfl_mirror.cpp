// Read-only mirror: DRFL monitoring stream -> /joint_states (+ /gripper_state).
// Never requests control authority and never sends commands, so it can run next to the DRFL GUI
// (the controller accepts several monitoring clients). Gripper: SCHUNK EGP-C 40 M1013 on the flange I/O
// (DO1 close, DO2 open, DI1 closed-end, DI2 open-end).
#include "DRFLEx.h"

#include <array>
#include <chrono>
#include <cmath>
#include <memory>
#include <mutex>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/string.hpp>

using namespace DRAFramework;
using namespace std::chrono_literals;

namespace {
std::mutex g_mtx;
std::array<float, 6> g_q{};
unsigned char g_di[6]{}, g_do[6]{};
std::chrono::steady_clock::time_point g_stamp{};
}  // namespace

class Mirror : public rclcpp::Node {
public:
    Mirror() : Node("drfl_mirror") {
        ip_ = declare_parameter<std::string>("ip", "192.168.137.100");
        const double rate = declare_parameter<double>("rate", 50.0);
        js_pub_ = create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);
        grip_pub_ = create_publisher<std_msgs::msg::String>("gripper_state", 10);
        robot_.set_on_monitoring_data_ex([](const LPMONITORING_DATA_EX d) {
            std::lock_guard<std::mutex> lk(g_mtx);
            for (int i = 0; i < 6; ++i) {
                g_q[i] = d->_tCtrl._tJoint._fActualPos[i];
                g_di[i] = d->_tMisc._iActualDI[i];
                g_do[i] = d->_tMisc._iActualDO[i];
            }
            g_stamp = std::chrono::steady_clock::now();
        });
        robot_.set_on_disconnected([]() {});
        connect();
        timer_ = create_wall_timer(std::chrono::duration<double>(1.0 / rate), [this] { tick(); });
    }
    ~Mirror() override { robot_.close_connection(); }

private:
    void connect() {
        connected_ = robot_.open_connection(ip_, 12345);
        if (connected_) {
            robot_.setup_monitoring_version(1);
            RCLCPP_INFO(get_logger(), "connected (read-only) to %s", ip_.c_str());
        } else {
            RCLCPP_WARN(get_logger(), "cannot connect to %s, retrying", ip_.c_str());
        }
        last_try_ = now();
    }

    void tick() {
        std::array<float, 6> q;
        unsigned char di[6], dout[6];
        std::chrono::steady_clock::time_point st;
        {
            std::lock_guard<std::mutex> lk(g_mtx);
            q = g_q;
            std::copy(g_di, g_di + 6, di);
            std::copy(g_do, g_do + 6, dout);
            st = g_stamp;
        }
        const bool fresh = st != std::chrono::steady_clock::time_point{} &&
                           std::chrono::steady_clock::now() - st < 500ms;
        if (!fresh) {
            if ((now() - last_try_).seconds() > 3.0) {
                robot_.close_connection();
                connect();
            }
            return;  // never publish stale / all-zero joints
        }
        sensor_msgs::msg::JointState js;
        js.header.stamp = now();
        for (int i = 0; i < 6; ++i) {
            js.name.push_back("joint_" + std::to_string(i + 1));
            js.position.push_back(q[i] * M_PI / 180.0);
        }
        js_pub_->publish(js);
        std_msgs::msg::String g;
        if (di[0] && di[1]) g.data = "error";
        else if (di[1]) g.data = "open";
        else if (di[0]) g.data = "closed";
        else if (dout[0] && !dout[1]) g.data = "holding";
        else g.data = "middle";
        grip_pub_->publish(g);
    }

    CDRFLEx robot_;
    std::string ip_;
    bool connected_ = false;
    rclcpp::Time last_try_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr js_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr grip_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Mirror>());
    rclcpp::shutdown();
    return 0;
}
