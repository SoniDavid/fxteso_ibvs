#include "body_wrench_system.hh"

#include <gz/plugin/Register.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/Link.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/components/ExternalWorldWrenchCmd.hh>
#include <gz/sim/components/Model.hh>
#include <gz/sim/components/Name.hh>
#include <gz/msgs/double.pb.h>
#include <gz/msgs/vector3d.pb.h>
#include <gz/msgs/odometry.pb.h>
#include <gz/msgs/Utility.hh>
#include <gz/common/Console.hh>

#include <cmath>

using namespace fxteso_ibvs;

void BodyWrench::Configure(const gz::sim::Entity &entity,
                           const std::shared_ptr<const sdf::Element> &sdf,
                           gz::sim::EntityComponentManager &ecm,
                           gz::sim::EventManager &)
{
  // "plant" writes the wrench; "echo" is a later instance that reads back the accumulated
  // total. A world plugin: <plugin> inside <include> is ignored by gz-sim 8.
  this->echo = sdf->Get<std::string>("mode", "plant").first == "echo";
  this->modelName = sdf->Get<std::string>("model_name", "F450").first;
  this->linkName = sdf->Get<std::string>("link_name", "base_link").first;

  gz::sim::Model model(entity);
  if (model.Valid(ecm))
    this->linkEntity = model.LinkByName(ecm, this->linkName);

  this->maxForce = sdf->Get<double>("max_force", this->maxForce).first;
  this->maxTorque = sdf->Get<double>("max_torque", this->maxTorque).first;

  // Holds hover until the controllers come up, mirroring uav_dynamics' 5 s hold.
  this->thrust = sdf->Get<double>("initial_thrust", 0.0).first;

  const std::string prefix = sdf->Get<std::string>("topic_prefix", "/F450").first;

  this->node.Subscribe<gz::msgs::Double>(prefix + "/thrust",
      [this](const gz::msgs::Double &msg)
      {
        const double v = msg.data();
        if (!std::isfinite(v) || std::abs(v) > this->maxForce)
        {
          if (!this->warned)
          {
            gzerr << "BodyWrench: rejecting thrust [" << v << "], holding last good value. "
                  << "This is usually a diverged controller.\n";
            this->warned = true;
          }
          return;
        }
        std::lock_guard<std::mutex> lock(this->mutex);
        this->thrust = v;
      });

  this->node.Subscribe<gz::msgs::Vector3d>(prefix + "/torques",
      [this](const gz::msgs::Vector3d &msg)
      {
        const gz::math::Vector3d v(msg.x(), msg.y(), msg.z());
        if (!this->Accept(v, this->maxTorque, "torque")) return;
        std::lock_guard<std::mutex> lock(this->mutex);
        this->torque = v;
      });

  this->node.Subscribe<gz::msgs::Vector3d>(prefix + "/disturbance",
      [this](const gz::msgs::Vector3d &msg)
      {
        const gz::math::Vector3d v(msg.x(), msg.y(), msg.z());
        if (!this->Accept(v, this->maxForce, "disturbance")) return;
        std::lock_guard<std::mutex> lock(this->mutex);
        this->disturbance = v;
      });

  // Straight out of the ECM, not the stock OdometryPublisher: that differences poses, and
  // the resulting noise is indistinguishable from the disturbance the observer estimates.
  if (this->echo)
  {
    // Everything acting on the airframe that is not commanded thrust: the ROS-side
    // /disturbances plus whatever WindEffects applied. This is the ground truth the FxTESO
    // estimate is plotted against - without it a Gazebo wind plugin is invisible.
    this->windPub = this->node.Advertise<gz::msgs::Vector3d>(prefix + "/external_force");
  }
  else
  {
    this->statePub = this->node.Advertise<gz::msgs::Odometry>(prefix + "/state");
  }

  const double rate = sdf->Get<double>("state_publish_rate", 100.0).first;
  if (rate > 0.0)
  {
    this->stateInterval = std::chrono::duration_cast<std::chrono::steady_clock::duration>(
        std::chrono::duration<double>(1.0 / rate));
  }

  gzmsg << "BodyWrench on [" << this->modelName << "/" << this->linkName << "], commands on ["
        << prefix << "/{thrust,torques,disturbance}], state on [" << prefix << "/state], "
        << "initial thrust " << this->thrust << " N\n";
}

bool BodyWrench::Resolve(gz::sim::EntityComponentManager &ecm)
{
  const gz::sim::Entity modelEntity = ecm.EntityByComponents(
      gz::sim::components::Name(this->modelName), gz::sim::components::Model());
  if (modelEntity == gz::sim::kNullEntity)
    return false;

  this->linkEntity = gz::sim::Model(modelEntity).LinkByName(ecm, this->linkName);
  if (this->linkEntity == gz::sim::kNullEntity)
  {
    gzerr << "BodyWrench: model [" << this->modelName << "] has no link ["
          << this->linkName << "].\n";
    return false;
  }

  gz::sim::Link(this->linkEntity).EnableVelocityChecks(ecm, true);
  gzmsg << "BodyWrench: attached to [" << this->modelName << "/" << this->linkName << "]\n";
  return true;
}

bool BodyWrench::Accept(const gz::math::Vector3d &v, double limit, const char *what)
{
  if (v.IsFinite() && v.Length() <= limit)
    return true;

  if (!this->warned)
  {
    gzerr << "BodyWrench: rejecting " << what << " [" << v << "], holding last good value. "
          << "This is usually a diverged controller.\n";
    this->warned = true;
  }
  return false;
}

void BodyWrench::PreUpdate(const gz::sim::UpdateInfo &info,
                           gz::sim::EntityComponentManager &ecm)
{
  if (info.paused)
    return;

  if (this->linkEntity == gz::sim::kNullEntity && !this->Resolve(ecm))
    return;

  gz::sim::Link link(this->linkEntity);
  const auto pose = link.WorldPose(ecm);
  if (!pose.has_value())
    return;

  double t;
  gz::math::Vector3d tau, dist;
  {
    std::lock_guard<std::mutex> lock(this->mutex);
    t = this->thrust;
    tau = this->torque;
    dist = this->disturbance;
  }

  const gz::math::Quaterniond &R = pose->Rot();

  // The plant's "-thrust*e3" in a z-down body frame is +z in this FLU model.
  gz::math::Vector3d force = R.RotateVector(gz::math::Vector3d(0, 0, t));

  // uav_dynamics applies "-R^T d", i.e. an inertial -d; NED -> world flips y and z.
  force += gz::math::Vector3d(-dist.X(), dist.Y(), dist.Z());

  const gz::math::Vector3d torqueWorld =
      R.RotateVector(gz::math::Vector3d(tau.X(), -tau.Y(), -tau.Z()));

  auto *comp = ecm.Component<gz::sim::components::ExternalWorldWrenchCmd>(this->linkEntity);
  if (comp == nullptr)
  {
    ecm.CreateComponent(this->linkEntity, gz::sim::components::ExternalWorldWrenchCmd());
    comp = ecm.Component<gz::sim::components::ExternalWorldWrenchCmd>(this->linkEntity);
  }


  if (this->echo)
  {
    // Total minus commanded thrust: what the observer should be estimating.
    const gz::math::Vector3d total(comp->Data().force().x(), comp->Data().force().y(),
                                   comp->Data().force().z());
    const gz::math::Vector3d thrustWorld = R.RotateVector(gz::math::Vector3d(0, 0, t));
    if (info.simTime >= this->nextState)
    {
      this->nextState = info.simTime + this->stateInterval;
      gz::msgs::Vector3d msg;
      gz::msgs::Set(&msg, total - thrustWorld);
      this->windPub.Publish(msg);
    }
    return;
  }

  gz::msgs::Set(comp->Data().mutable_force(), force);
  gz::msgs::Set(comp->Data().mutable_torque(), torqueWorld);

  this->PublishState(info, ecm, link, *pose);
}

void BodyWrench::PublishState(const gz::sim::UpdateInfo &info,
                              const gz::sim::EntityComponentManager &ecm,
                              gz::sim::Link &link,
                              const gz::math::Pose3d &pose)
{
  if (info.simTime < this->nextState)
    return;
  this->nextState = info.simTime + this->stateInterval;

  const auto vLin = link.WorldLinearVelocity(ecm);
  const auto vAng = link.WorldAngularVelocity(ecm);
  if (!vLin.has_value() || !vAng.has_value())
    return;

  gz::msgs::Odometry msg;
  msg.mutable_header()->mutable_stamp()->CopyFrom(gz::msgs::Convert(info.simTime));
  gz::msgs::Set(msg.mutable_pose(), pose);

  // nav_msgs convention: twist is in the child (body) frame.
  gz::msgs::Set(msg.mutable_twist()->mutable_linear(),
                pose.Rot().RotateVectorReverse(*vLin));
  gz::msgs::Set(msg.mutable_twist()->mutable_angular(),
                pose.Rot().RotateVectorReverse(*vAng));

  this->statePub.Publish(msg);
}

GZ_ADD_PLUGIN(fxteso_ibvs::BodyWrench,
              gz::sim::System,
              fxteso_ibvs::BodyWrench::ISystemConfigure,
              fxteso_ibvs::BodyWrench::ISystemPreUpdate)

GZ_ADD_PLUGIN_ALIAS(fxteso_ibvs::BodyWrench, "fxteso_ibvs::BodyWrench")
