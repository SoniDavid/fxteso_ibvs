#ifndef FXTESO_IBVS_BODY_WRENCH_SYSTEM_HH
#define FXTESO_IBVS_BODY_WRENCH_SYSTEM_HH

#include <gz/sim/System.hh>
#include <gz/sim/Entity.hh>
#include <gz/sim/Link.hh>
#include <gz/transport/Node.hh>
#include <gz/math/Vector3.hh>
#include <gz/math/Pose3.hh>

#include <chrono>
#include <memory>
#include <mutex>
#include <string>

namespace quad_gz_sim
{
/// \brief Applies thrust, body torque and an inertial disturbance to a link as an external
/// wrench, so DART integrates the aircraft instead of uav_dynamics.cpp. Commands arrive on
/// <topic_prefix>/{thrust,torques,disturbance}, bridged from ROS. See docs/gazebo-plant.md.
class BodyWrench : public gz::sim::System,
                   public gz::sim::ISystemConfigure,
                   public gz::sim::ISystemPreUpdate
{
public:
  void Configure(const gz::sim::Entity &entity,
                 const std::shared_ptr<const sdf::Element> &sdf,
                 gz::sim::EntityComponentManager &ecm,
                 gz::sim::EventManager &eventMgr) override;

  void PreUpdate(const gz::sim::UpdateInfo &info,
                 gz::sim::EntityComponentManager &ecm) override;

private:
  /// \brief Rejects non-finite or absurd commands so NaN cannot reach DART.
  bool Accept(const gz::math::Vector3d &v, double limit, const char *what);

  /// \brief Finds the target link; deferred, as a world plugin loads before the models.
  bool Resolve(gz::sim::EntityComponentManager &ecm);

  void PublishState(const gz::sim::UpdateInfo &info,
                    const gz::sim::EntityComponentManager &ecm,
                    gz::sim::Link &link,
                    const gz::math::Pose3d &pose);

  std::string modelName{"F450"};
  std::string linkName{"base_link"};
  gz::sim::Entity linkEntity{gz::sim::kNullEntity};
  gz::transport::Node node;
  gz::transport::Node::Publisher statePub;
  gz::transport::Node::Publisher windPub;

  /// \brief Sim time of the next state publication, and the interval.
  std::chrono::steady_clock::duration nextState{0};
  std::chrono::steady_clock::duration stateInterval{std::chrono::milliseconds(10)};

  std::mutex mutex;
  double thrust{0.0};
  gz::math::Vector3d torque{0, 0, 0};
  gz::math::Vector3d disturbance{0, 0, 0};

  double maxForce{1.0e4};
  double maxTorque{1.0e4};
  bool warned{false};
  bool echo{false};
};
}  // namespace quad_gz_sim

#endif
