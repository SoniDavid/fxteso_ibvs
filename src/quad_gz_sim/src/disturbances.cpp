// Disturbance-force generator. Publishes /disturbances at 100 Hz as an INERTIAL-FRAME FORCE
// IN NEWTONS, applied by uav_dynamics.cpp as "- R(eta)' * dist" (note the sign).
// Tuning: config/disturbances.yaml. 

//Including ROS libraries
#include <rclcpp/rclcpp.hpp>
#include "quad_common/sim_rate.hpp"
#include <chrono>
#include <std_msgs/msg/float64.hpp>
#include <geometry_msgs/msg/vector3.hpp>
//Including C++ nominal libraries
#include <iostream>
#include <math.h>
#include <vector>
#include <fstream>
#include <string>
#include <sstream>
#include <random>
//Including Eigen library
#include <eigen3/Eigen/Dense>

using namespace std;

static const double kRate = 100.0;
static const double kStep = 1.0 / kRate;
static const double kAirDensity = 1.225;   // kg/m^3, sea level

// Quad's inertial velocity, for the airspeed of the 'wind' profile.
Eigen::Vector3f quad_vel_IF(0, 0, 0);

void quadVelCallback(const geometry_msgs::msg::Vector3::ConstSharedPtr v)
{
    quad_vel_IF(0) = v->x;
    quad_vel_IF(1) = v->y;
    quad_vel_IF(2) = v->z;
}

// "gust", "step,gust", "none" - the profiles compose, so this is a set, not an enum.
static bool hasProfile(const string &profiles, const string &name)
{
    stringstream stream(profiles);
    string item;
    while (getline(stream, item, ','))
    {
        // trim
        const size_t b = item.find_first_not_of(" \t");
        if (b == string::npos)
            continue;
        const size_t e = item.find_last_not_of(" \t");
        if (item.substr(b, e - b + 1) == name)
            return true;
    }
    return false;
}

static Eigen::Vector3d toVec3(const vector<double> &v, const char *what,
                              const rclcpp::Logger &log)
{
    if (v.size() != 3)
    {
        RCLCPP_FATAL(log, "Parameter '%s' needs exactly 3 elements, got %zu.", what, v.size());
        exit(1);
    }
    return Eigen::Vector3d(v[0], v[1], v[2]);
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("disturbances");
    const auto log = node->get_logger();

    fxteso::SimRate loop_rate(node, kRate);

    auto disturbances_pub = node->create_publisher<geometry_msgs::msg::Vector3>("disturbances", 100);
    auto quad_vel_sub = node->create_subscription<geometry_msgs::msg::Vector3>(
        "quad_velocity", 1, quadVelCallback);
    geometry_msgs::msg::Vector3 disturbances_var;

    // Which generators run. Comma-separated so they combine; "none" publishes 0 N.
    const string profiles = node->declare_parameter<string>("profile", "gust");

    // Constant bias, added whenever anything at all is active.
    const Eigen::Vector3d mean =
        toVec3(node->declare_parameter<vector<double>>("mean", {0.0, 0.0, 0.0}), "mean", log);

    // Rectangular pulse, the thesis' canonical step test. t = 0 is the end of the 8 s hold.
    const Eigen::Vector3d step_amplitude = toVec3(
        node->declare_parameter<vector<double>>("step_amplitude", {0.3, 0.3, 0.3}),
        "step_amplitude", log);
    const double step_start = node->declare_parameter<double>("step_start", 22.0);
    const double step_end = node->declare_parameter<double>("step_end", 25.0);

    // Random force, Ornstein-Uhlenbeck: stationary, band-limited, zero-mean.
    // sigma in N, tau in s. State-independent, so its truth signal is clean.
    const Eigen::Vector3d gust_sigma = toVec3(
        node->declare_parameter<vector<double>>("gust_sigma", {0.4, 0.4, 0.2}),
        "gust_sigma", log);
    const double gust_tau = node->declare_parameter<double>("gust_tau", 1.5);
    const int seed = node->declare_parameter<int>("seed", 0);

    // Aerodynamic drag: F = 0.5*rho*Cd*A*|v_rel|*v_rel, v_rel = v_wind - v_quad.
    // State-dependent, unlike the others.
    const Eigen::Vector3d wind_velocity = toVec3(
        node->declare_parameter<vector<double>>("wind_velocity", {0.0, 0.0, 0.0}),
        "wind_velocity", log);
    const double drag_cd_a = node->declare_parameter<double>("drag_cd_a", 0.15);

    // OU turbulence on the wind velocity (m/s), applied before the drag law - this is what
    // makes 'wind' gust rather than blow steadily. Zero means steady.
    const Eigen::Vector3d wind_turbulence = toVec3(
        node->declare_parameter<vector<double>>("wind_turbulence", {0.0, 0.0, 0.0}),
        "wind_turbulence", log);
    const double wind_turbulence_tau = node->declare_parameter<double>("wind_turbulence_tau", 2.0);

    // Replay a recorded profile, one "x,y,z" row per 10 ms step. An unopenable path aborts:
    // the original silently published 0 N for entire runs.
    const string csv = node->declare_parameter<string>("csv", "");

    const bool use_step = hasProfile(profiles, "step");
    const bool use_gust = hasProfile(profiles, "gust");
    const bool use_wind = hasProfile(profiles, "wind");
    const bool use_csv = hasProfile(profiles, "csv");
    const bool use_none = hasProfile(profiles, "none");

    if (!use_step && !use_gust && !use_wind && !use_csv && !use_none)
    {
        RCLCPP_FATAL(log, "Unknown profile '%s'. Expected a comma-separated subset of "
                          "none,step,gust,wind,csv.", profiles.c_str());
        return 1;
    }

    ifstream myFile;
    if (use_csv)
    {
        if (csv.empty())
        {
            RCLCPP_FATAL(log, "profile includes 'csv' but the 'csv' parameter is empty. "
                              "Point it at a profile file.");
            return 1;
        }
        myFile.open(csv);
        if (!myFile.is_open())
        {
            RCLCPP_FATAL(log, "Cannot open the disturbance profile '%s'.", csv.c_str());
            return 1;
        }
        string header;
        getline(myFile, header);   // discard the header row
    }

    // Logged so every run is self-describing.
    RCLCPP_INFO(log,
                "Disturbance profile '%s': mean=[%.3f %.3f %.3f] N"
                "%s%s%s%s",
                profiles.c_str(), mean(0), mean(1), mean(2),
                use_step ? "; step" : "", use_gust ? "; gust" : "",
                use_wind ? "; wind" : "", use_csv ? "; csv" : "");
    if (use_step)
        RCLCPP_INFO(log, "  step: [%.3f %.3f %.3f] N over t in [%.2f, %.2f] s",
                    step_amplitude(0), step_amplitude(1), step_amplitude(2),
                    step_start, step_end);
    if (use_gust)
        RCLCPP_INFO(log, "  gust: sigma=[%.3f %.3f %.3f] N, tau=%.2f s, seed=%d",
                    gust_sigma(0), gust_sigma(1), gust_sigma(2), gust_tau, seed);
    if (use_wind)
    {
        RCLCPP_INFO(log, "  wind: v=[%.3f %.3f %.3f] m/s, Cd*A=%.3f m^2",
                    wind_velocity(0), wind_velocity(1), wind_velocity(2), drag_cd_a);
        if (wind_turbulence.norm() > 0.0)
            RCLCPP_INFO(log, "  wind turbulence: sigma=[%.3f %.3f %.3f] m/s, tau=%.2f s, "
                             "seed=%d", wind_turbulence(0), wind_turbulence(1),
                        wind_turbulence(2), wind_turbulence_tau, seed);
        else
            RCLCPP_INFO(log, "  wind turbulence: off (steady wind). Set 'wind_turbulence' "
                             "to make it gust.");
    }

    std::mt19937 rng(static_cast<uint32_t>(seed));
    std::normal_distribution<double> gaussian(0.0, 1.0);
    Eigen::Vector3d gust(0, 0, 0);
    Eigen::Vector3d wind_turb(0, 0, 0);

    // 8 s hold: the plant must be flying before it is pushed. t is measured from here.
    disturbances_var.x = 0;
    disturbances_var.y = 0;
    disturbances_var.z = 0;
    disturbances_pub->publish(disturbances_var);
    loop_rate.sleepFor(8.0);

    long i = 0;
    while (rclcpp::ok())
    {
        const double t = i * kStep;
        Eigen::Vector3d dist(0, 0, 0);

        if (!use_none)
            dist += mean;

        if (use_step && t >= step_start && t <= step_end)
            dist += step_amplitude;

        if (use_gust)
        {
            // d <- d - (d/tau)*dt + sigma*sqrt(2*dt/tau)*N(0,1)
            const double decay = kStep / gust_tau;
            const double diffusion = sqrt(2.0 * kStep / gust_tau);
            for (int k = 0; k < 3; k++)
                gust(k) += -gust(k) * decay + gust_sigma(k) * diffusion * gaussian(rng);
            dist += gust;
        }

        if (use_wind)
        {
            if (wind_turbulence.norm() > 0.0)
            {
                const double decay = kStep / wind_turbulence_tau;
                const double diffusion = sqrt(2.0 * kStep / wind_turbulence_tau);
                for (int k = 0; k < 3; k++)
                    wind_turb(k) += -wind_turb(k) * decay
                                    + wind_turbulence(k) * diffusion * gaussian(rng);
            }
            const Eigen::Vector3d v_rel =
                (wind_velocity + wind_turb) - quad_vel_IF.cast<double>();
            // MINUS: uav_dynamics applies "- dist", so +F_drag would turn the -v_quad term
            // into negative damping and diverge at any wind speed.
            dist -= 0.5 * kAirDensity * drag_cd_a * v_rel.norm() * v_rel;
        }

        if (use_csv)
        {
            string line;
            if (getline(myFile, line))
            {
                stringstream stream(line);
                string x, y, z;
                getline(stream, x, ',');
                getline(stream, y, ',');
                getline(stream, z, ',');
                dist += Eigen::Vector3d(stod(x), stod(y), stod(z));
            }
            // Past end of file this contributes zero; other generators keep running.
        }

        disturbances_var.x = dist(0);
        disturbances_var.y = dist(1);
        disturbances_var.z = dist(2);
        disturbances_pub->publish(disturbances_var);

        i++;
        loop_rate.sleep();
    }

    if (myFile.is_open())
        myFile.close();

    rclcpp::shutdown();

    return 0;
}
