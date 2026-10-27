// Euler angle unwrapping for every state adapter: the control stack needs unbounded angles.

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

	// Take the next sample as a real jump, not a wrap (an EKF2 heading reset); turns are kept.
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
