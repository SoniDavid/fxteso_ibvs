// PX4 uXRCE-DDS topic naming.
//
// Since v1.16 the client appends _v<N> when a message's MESSAGE_VERSION is non-zero
// (uxrce_dds_client/utilities.hpp:35), so VehicleStatus is /fmu/out/vehicle_status_v4 while
// VehicleOdometry is plain. Subscribing to the unversioned name of a versioned message is
// silent: the subscription is created and never receives anything.
//
// Reading the version off the type means a px4_msgs bump moves our topic with it.

#ifndef QUAD_PX4__PX4_TOPIC_HPP_
#define QUAD_PX4__PX4_TOPIC_HPP_

#include <cstdint>
#include <string>
#include <type_traits>

namespace quad_px4
{

// Messages generated from a .msg without a MESSAGE_VERSION constant have no such member;
// those are unversioned, which is the same as version 0.
template <typename T, typename = void>
struct MessageVersion : std::integral_constant<uint32_t, 0> {};

template <typename T>
struct MessageVersion<T, std::void_t<decltype(T::MESSAGE_VERSION)>>
: std::integral_constant<uint32_t, T::MESSAGE_VERSION> {};

template <typename T>
inline std::string px4Topic(const std::string &base)
{
	constexpr uint32_t version = MessageVersion<T>::value;
	return version == 0 ? base : base + "_v" + std::to_string(version);
}

}  // namespace quad_px4

#endif  // QUAD_PX4__PX4_TOPIC_HPP_
