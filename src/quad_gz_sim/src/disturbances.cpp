// Disturbance-force generator. Publishes /disturbances at 100 Hz as an INERTIAL-FRAME FORCE
// IN NEWTONS, applied by uav_dynamics.cpp as "- R(eta)' * dist" (note the sign).
// Tuning: config/disturbances.yaml. 

//Including ROS libraries
#include <rclcpp/rclcpp.hpp>
#include "quad_common/sim_rate.hpp"
#include <chrono>
#include <std_msgs/msg/float64.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
//Including C++ nominal libraries
#include <iostream>
#include <math.h>
#include <vector>
#include <fstream>
#include <string>
#include <sstream>
#include <random>
#include <algorithm>
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

// --- thesis Table 5.2 wind ------------------------------------------------------------------
// The thesis' own wind, which nothing in this project had ever flown: the mean reverses
// direction, carries a -4 m/s downdraft, and is removed at t > 190 s so convergence can be
// shown afterwards. Held in the source rather than the YAML because it is a specification to
// reproduce - a run with edited intervals is not Table 5.2 - while the knobs that are
// legitimately variable (altitude, turbulence scale, on/off times) are parameters.
struct MeanInterval { double until; double mu[3]; };      // s, m/s, inertial frame
static const MeanInterval kTable52Mean[] = {
    { 55.0, { 2.0,  2.0,  0.0}},
    {100.0, { 3.0, -2.0, -1.0}},
    {155.0, {-2.0,  1.0, -4.0}},
    {190.0, { 0.0,  1.0,  1.0}},
};
// Turbulence intensity by interval, as W20: 15 kn light, 30 kn moderate. Its own schedule,
// which does NOT line up with the mean's - see the thesis table.
struct TurbInterval { double until; double w20; };        // s, m/s
static const TurbInterval kTable52Turb[] = {
    { 70.0, 15.43},   // moderate
    {150.0,  7.71},   // light
    {190.0, 15.43},   // moderate
};

// Von Karman shaping filters, Eqs. 2.27-2.29, driven by white noise of PSD pi - NOT 1.
// The thesis' spectra are one-sided in Omega and integrate to sigma^2 over [0, inf), while an
// Euler simulation of a filter driven by unit noise realises (1/2pi) times the integral over
// (-inf, inf). The factor is exactly pi. With it the analytic H2 norm of both filters below is
// 0.98*sigma, which is the rational approximations' own error and not a bug.
static const double kNoisePsd = M_PI;
// The vertical filter's fastest pole is ~15 rad/s against this node's 10 ms tick, which plain
// Euler will not hold. Integrate the filter states in substeps.
static const int kTurbSubsteps = 10;

// (1 + 0.25 T s) / (1 + 1.357 T s + 0.1987 T^2 s^2), controllable canonical form. T = L/V.
static double vonKarmanLong(double x[3], double tau, double gain, double dt, double w)
{
    const double a2 = 0.1987 * tau * tau, a1 = 1.357 * tau;
    const double x2d = (-x[0] - a1 * x[1]) / a2 + w;
    const double x1 = x[0] + x[1] * dt;
    x[1] += x2d * dt;
    x[0] = x1;
    return gain / a2 * (x[0] + 0.25 * tau * x[1]);
}

// (1 + 2.7478 T s + 0.3398 T^2 s^2) / (1 + 2.9958 T s + 1.9754 T^2 s^2 + 0.1539 T^3 s^3).
static double vonKarmanLat(double x[3], double tau, double gain, double dt, double w)
{
    const double d3 = 0.1539 * tau * tau * tau, d2 = 1.9754 * tau * tau, d1 = 2.9958 * tau;
    const double x3d = (-x[0] - d1 * x[1] - d2 * x[2]) / d3 + w;
    const double x1 = x[0] + x[1] * dt;
    const double x2 = x[1] + x[2] * dt;
    x[2] += x3d * dt;
    x[1] = x2;
    x[0] = x1;
    return gain / d3 * (x[0] + 2.7478 * tau * x[1] + 0.3398 * tau * tau * x[2]);
}

// Eq. 2.25, per axis, as a force to SUBTRACT from dist: uav_dynamics applies "- dist", so
// adding +F_drag would turn the -v_quad term into negative damping and diverge at any wind.
static Eigen::Vector3d dragForce(const Eigen::Vector3d &v_rel, const Eigen::Vector3d &cd_a)
{
    Eigen::Vector3d f;
    for (int k = 0; k < 3; k++)
        f(k) = 0.5 * kAirDensity * cd_a(k) * v_rel(k) * fabs(v_rel(k));
    return f;
}

// Set by the first desired_attitude, the same handover signal target_position waits on.
bool control_started = false;

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("disturbances");
    const auto log = node->get_logger();

    fxteso::SimRate loop_rate(node, kRate);

    auto disturbances_pub = node->create_publisher<geometry_msgs::msg::Vector3>("disturbances", 100);
    // Under px4 the aircraft is still on the ground 8 s in; wait for the controller handover
    // instead. Default false so the analytic and gazebo plants keep their old timing exactly.
    const bool hold_until_control = node->declare_parameter<bool>("hold_until_control", false);
    const double settle = node->declare_parameter<double>("settle", 8.0);
    auto ctrl_sub = node->create_subscription<geometry_msgs::msg::Quaternion>(
        "desired_attitude", 1,
        [](const geometry_msgs::msg::Quaternion::ConstSharedPtr) { control_started = true; });
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

    // Random force, Ornstein-Uhlenbeck: stationary, band-limited, zero-mean. sigma in N, tau
    // in s. State-independent, so its truth signal is clean.
    const double gust_scale = node->declare_parameter<double>("gust_scale", 1.0);
    const Eigen::Vector3d gust_sigma = gust_scale * toVec3(
        node->declare_parameter<vector<double>>("gust_sigma", {0.4, 0.4, 0.2}),
        "gust_sigma", log);
    // Correlation time of the OU gust. Sweepable because the FxTESO's estimate was measured
    // trailing the disturbance by ~1.5 s - exactly this value - and whether the lag follows
    // tau or stays put is what separates an observer-bandwidth limit from a coincidence.
    const double gust_tau = node->declare_parameter<double>("gust_tau", 1.5);
    const int seed = node->declare_parameter<int>("seed", 0);

    // Aerodynamic drag, thesis Eq. 2.25, PER AXIS: -0.5*rho*drag_cd_a*v_rel^2*sign(v_rel) with
    // v_rel = v_wind - v_quad. Isotropic |v_rel|*v_rel differs by up to sqrt(2) on a diagonal.
    const double wind_scale = node->declare_parameter<double>("wind_scale", 1.0);
    const Eigen::Vector3d wind_velocity = wind_scale * toVec3(
        node->declare_parameter<vector<double>>("wind_velocity", {0.0, 0.0, 0.0}),
        "wind_velocity", log);
    const Eigen::Vector3d drag_cd_a = toVec3(
        node->declare_parameter<vector<double>>("drag_cd_a", {0.03, 0.03, 0.10}),
        "drag_cd_a", log);

    // OU turbulence on the wind velocity (m/s), applied before the drag law - this is what
    // makes 'wind' gust rather than blow steadily. Zero means steady.
    const Eigen::Vector3d wind_turbulence = wind_scale * toVec3(
        node->declare_parameter<vector<double>>("wind_turbulence", {0.0, 0.0, 0.0}),
        "wind_turbulence", log);
    const double wind_turbulence_tau = node->declare_parameter<double>("wind_turbulence_tau", 2.0);

    // 'table52': the thesis' own wind (above). Superseding rather than extending 'wind', so
    // every bag recorded against wind_velocity stays reproducible.
    // turbulence_scale multiplies the Von Karman sigmas only: 0 leaves the mean schedule alone,
    // which separates what the structure costs from what the gusting costs.
    const double turbulence_scale = node->declare_parameter<double>("turbulence_scale", 1.0);
    // Eqs. 2.30-2.33 are altitude-dependent; the thesis evaluates them at the servoing depth.
    const double table52_altitude = node->declare_parameter<double>("table52_altitude", 2.5);
    const double table52_start = node->declare_parameter<double>("table52_start", 10.0);
    const double table52_end = node->declare_parameter<double>("table52_end", 190.0);
    // The table's means are themselves white Gaussian signals, s.d. sigma_g, held at 10 Hz.
    const double table52_sigma_g = node->declare_parameter<double>("table52_sigma_g", 0.1);
    const double table52_mean_rate = node->declare_parameter<double>("table52_mean_rate", 10.0);
    // The Von Karman coefficients all scale with airspeed, so a step in V steps every
    // coefficient while the filter states stay where they were, and the output jumps. Measured
    // before this existed: a 40 m/s spike in the first 0.1 s of every run, which through the z
    // drag is ~98 N on a 2 kg aircraft. Low-passing V leaves the coefficients only ever
    // drifting, slowly enough that the filter tracks quasi-statically.
    const double table52_airspeed_tau =
        node->declare_parameter<double>("table52_airspeed_tau", 10.0);

    // Replay a recorded profile, one "x,y,z" row per 10 ms step. An unopenable path aborts:
    // the original silently published 0 N for entire runs.
    const string csv = node->declare_parameter<string>("csv", "");

    const bool use_step = hasProfile(profiles, "step");
    const bool use_gust = hasProfile(profiles, "gust");
    const bool use_wind = hasProfile(profiles, "wind");
    const bool use_csv = hasProfile(profiles, "csv");
    const bool use_table52 = hasProfile(profiles, "table52");
    const bool use_none = hasProfile(profiles, "none");

    if (!use_step && !use_gust && !use_wind && !use_csv && !use_table52 && !use_none)
    {
        RCLCPP_FATAL(log, "Unknown profile '%s'. Expected a comma-separated subset of "
                          "none,step,gust,wind,table52,csv.", profiles.c_str());
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
                "%s%s%s%s%s",
                profiles.c_str(), mean(0), mean(1), mean(2),
                use_step ? "; step" : "", use_gust ? "; gust" : "",
                use_wind ? "; wind" : "", use_table52 ? "; table52" : "",
                use_csv ? "; csv" : "");
    if (use_step)
        RCLCPP_INFO(log, "  step: [%.3f %.3f %.3f] N over t in [%.2f, %.2f] s",
                    step_amplitude(0), step_amplitude(1), step_amplitude(2),
                    step_start, step_end);
    if (use_gust)
        RCLCPP_INFO(log, "  gust: sigma=[%.4f %.4f %.4f] N (scale %.3f), tau=%.2f s, seed=%d",
                    gust_sigma(0), gust_sigma(1), gust_sigma(2), gust_scale, gust_tau, seed);
    if (use_wind)
    {
        RCLCPP_INFO(log, "  wind: v=[%.3f %.3f %.3f] m/s, Cd*A=[%.3f %.3f %.3f] m^2 "
                         "(thesis Eq. 2.25, per axis)",
                    wind_velocity(0), wind_velocity(1), wind_velocity(2),
                    drag_cd_a(0), drag_cd_a(1), drag_cd_a(2));
        if (wind_turbulence.norm() > 0.0)
            RCLCPP_INFO(log, "  wind turbulence: sigma=[%.3f %.3f %.3f] m/s, tau=%.2f s, "
                             "seed=%d", wind_turbulence(0), wind_turbulence(1),
                        wind_turbulence(2), wind_turbulence_tau, seed);
        else
            RCLCPP_INFO(log, "  wind turbulence: off (steady wind). Set 'wind_turbulence' "
                             "to make it gust.");
    }
    if (use_table52)
    {
        // Sigmas at the servoing altitude, Eqs. 2.30-2.33. Logged so the run is self-describing
        // and so a wrong altitude is visible without reading the code.
        const double h = pow(0.177 + 0.000823 * table52_altitude, 0.4);
        RCLCPP_INFO(log, "  table52: thesis Table 5.2 mean schedule, wind on t in [%.0f, %.0f] s"
                         ", sigma_g=%.2f m/s at %.0f Hz, z=%.2f m",
                    table52_start, table52_end, table52_sigma_g, table52_mean_rate,
                    table52_altitude);
        RCLCPP_INFO(log, "  table52 turbulence: scale %.2f, Von Karman sigma_u=%.2f (moderate) "
                         "/ %.2f (light) m/s, L_u=%.1f m, L_w=%.1f m",
                    turbulence_scale, turbulence_scale * 1.543 / h,
                    turbulence_scale * 0.771 / h,
                    table52_altitude / pow(0.177 + 0.000823 * table52_altitude, 1.2),
                    table52_altitude);
        if (turbulence_scale == 0.0)
            RCLCPP_INFO(log, "  table52 turbulence: OFF - the mean schedule alone.");
    }

    std::mt19937 rng(static_cast<uint32_t>(seed));
    std::normal_distribution<double> gaussian(0.0, 1.0);
    Eigen::Vector3d gust(0, 0, 0);
    Eigen::Vector3d wind_turb(0, 0, 0);
    // Von Karman filter states, one canonical triple per axis, and the 10 Hz mean draw.
    // The filters run from t=0 even though the wind is gated on at table52_start, so they are
    // stationary by the time their output is used - L_u/V is ~6 s of settling.
    double vk_state[3][3] = {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}};
    Eigen::Vector3d table52_mean_noise(0, 0, 0);
    // Seeded at the first interval's airspeed, not at zero: the warm-up must charge the filters
    // at the scale they will be used at.
    double table52_airspeed = Eigen::Vector3d(kTable52Mean[0].mu[0], kTable52Mean[0].mu[1],
                                              kTable52Mean[0].mu[2]).norm();

    // The plant must be FLYING before it is pushed, and t is measured from here. Under px4 a
    // fixed hold lands the disturbance on an aircraft still climbing, so wait for handover.
    disturbances_var.x = 0;
    disturbances_var.y = 0;
    disturbances_var.z = 0;
    disturbances_pub->publish(disturbances_var);

    if (hold_until_control)
    {
        RCLCPP_INFO(log, "Holding the disturbance at zero until desired_attitude appears.");
        while (rclcpp::ok() && !control_started)
        {
            disturbances_pub->publish(disturbances_var);
            loop_rate.sleep();
        }
        RCLCPP_INFO(log, "Handover seen - disturbance starts in %.1f s.", settle);
    }
    loop_rate.sleepFor(settle);

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
            dist -= dragForce((wind_velocity + wind_turb) - quad_vel_IF.cast<double>(),
                              drag_cd_a);
        }

        if (use_table52)
        {
            // Wind is removed outside [start, end] entirely, which is the point of the window:
            // the thesis switches it off at t > 190 s to show the state converging afterwards.
            const bool blowing = (t >= table52_start && t <= table52_end);

            // Evaluated even while the wind is off: the filters warm up through the whole
            // pre-start window, and warming them at a floored airspeed charges them at the
            // wrong scale - which is what produced the 40 m/s onset spike.
            Eigen::Vector3d v_mean(kTable52Mean[0].mu[0], kTable52Mean[0].mu[1],
                                   kTable52Mean[0].mu[2]);
            for (const MeanInterval &iv : kTable52Mean)
                if (t <= iv.until)
                {
                    v_mean = Eigen::Vector3d(iv.mu[0], iv.mu[1], iv.mu[2]);
                    break;
                }
            if (blowing)
            {
                // The table's means are white Gaussian at 10 Hz, so they are drawn on their own
                // clock and held, not resampled every 10 ms tick.
                const int decimation = max(1, (int)lround(kRate / table52_mean_rate));
                if (i % decimation == 0)
                    for (int k = 0; k < 3; k++)
                        table52_mean_noise(k) = table52_sigma_g * gaussian(rng);
                v_mean += table52_mean_noise;
            }

            // Airspeed drives the filter coefficients. Taken against the MEAN wind, not the
            // turbulent one, so the turbulence cannot feed back into its own bandwidth; floored
            // because V sits in every denominator below; low-passed so it never steps.
            table52_airspeed += ((v_mean - quad_vel_IF.cast<double>()).norm() - table52_airspeed)
                                * kStep / table52_airspeed_tau;
            const double V = max(0.5, table52_airspeed);
            const double hz = pow(0.177 + 0.000823 * table52_altitude, 0.4);   // Eq. 2.31
            const double hL = pow(0.177 + 0.000823 * table52_altitude, 1.2);   // Eq. 2.33
            double w20 = 0.0;
            for (const TurbInterval &iv : kTable52Turb)
                if (t <= iv.until)
                {
                    w20 = iv.w20;
                    break;
                }
            const double sigma_w = turbulence_scale * 0.1 * w20;               // Eq. 2.30
            const double sigma_u = sigma_w / hz;                               // Eq. 2.31
            const double L_w = table52_altitude;                               // Eq. 2.32
            const double L_u = table52_altitude / hL;                          // Eq. 2.33

            const double sub_dt = kStep / kTurbSubsteps;
            const double drive = sqrt(kNoisePsd / sub_dt);
            Eigen::Vector3d turb(0, 0, 0);
            for (int n = 0; n < kTurbSubsteps; n++)
            {
                turb(0) = vonKarmanLong(vk_state[0], L_u / V,
                                        sigma_u * sqrt(2.0 * L_u / (M_PI * V)),
                                        sub_dt, drive * gaussian(rng));
                turb(1) = vonKarmanLat(vk_state[1], L_u / V,
                                       sigma_u * sqrt(L_u / (M_PI * V)),
                                       sub_dt, drive * gaussian(rng));
                turb(2) = vonKarmanLat(vk_state[2], L_w / V,
                                       sigma_w * sqrt(L_w / (M_PI * V)),
                                       sub_dt, drive * gaussian(rng));
            }

            if (blowing)
                dist -= dragForce((v_mean + turb) - quad_vel_IF.cast<double>(), drag_cd_a);
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
