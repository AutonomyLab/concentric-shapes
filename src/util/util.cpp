#include "util/util.h"

#include "glm/glm.hpp"

float normalizeRadians(float radians)
{
	while(radians < -M_PI) radians += 2.0 * M_PI;
	while(radians > M_PI) radians -= 2.0 * M_PI;
	return radians;
}
