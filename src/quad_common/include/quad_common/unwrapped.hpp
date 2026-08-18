// Euler angle unwrapping, shared by every state adapter.
// The control stack needs unbounded angles; any quaternion-to-RPY conversion returns
// (-pi, pi].

#ifndef QUAD_COMMON__UNWRAPPED_HPP_
#define QUAD_COMMON__UNWRAPPED_HPP_

#include <cmath>

namespace fxteso
{

// A step above pi between consecutive samples is always a wrap, never motion.
class Unwrapped
{
public:
	double operator()(double raw)
	{
		if (this->started)
		{
			const double d = raw - this->prev;
			if (d > M_PI)
				this->turns -= 2.0 * M_PI;
			else if (d < -M_PI)
				this->turns += 2.0 * M_PI;
		}
		this->started = true;
		this->prev = raw;
		return raw + this->turns;
	}

	// Take the next sample as a real discontinuity, not a wrap: an estimator that corrects its
	// heading by more than pi (PX4's EKF2 while converging) must not bank a phantom turn.
	// Accumulated turns are kept, so a reset does not discard real winding.
	void reseed(double raw)
	{
		this->prev = raw;
		this->started = true;
	}

private:
	bool started{false};
	double prev{0.0};
	double turns{0.0};
};

}  // namespace fxteso

#endif  // QUAD_COMMON__UNWRAPPED_HPP_
