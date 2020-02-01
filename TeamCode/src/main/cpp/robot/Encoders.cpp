#include "Encoders.h"

#include "../Math.h"

namespace
{
    constexpr double DISTANCE_BETWEEN_ENCODER_WHEELS = 19.6125;
    constexpr double DISTANCE_TO_BACK_ENCODER = 14.2; // The distance to the tracking center
}

void Encoders::init() noexcept
{
    previousPose = RobotPose();
}

RobotPose Encoders::update(const RobotPose &pose, const double currentAngle) noexcept
{
    const double deltaLeft = pose.left - previousPose.left;
    const double deltaRight = pose.right - previousPose.right;
    const double deltaBack = pose.back - previousPose.back;
    const double deltaAngle = (deltaLeft - deltaRight) / DISTANCE_BETWEEN_ENCODER_WHEELS;

    previousPose = { deltaLeft, deltaRight, deltaBack };

    double newX = deltaBack;
    double newY = deltaRight;

    if (!Math::epsilonEquals(deltaAngle, 0.0))
    {
        const double sinDeltaAngle = sin(deltaAngle / 2);
        newX = 2 * sinDeltaAngle * (deltaBack / deltaAngle + DISTANCE_TO_BACK_ENCODER);
        newY = 2 * sinDeltaAngle * (deltaRight / deltaAngle + DISTANCE_BETWEEN_ENCODER_WHEELS / 2);
    }

    const double averageOrientation = -(currentAngle + deltaAngle / 2);

    // Calculate and update the position values
    // Rotate the cartesian coordinate system by transforming into polar form, adding the angle and
    // then transforming back into cartesian form.
    const double sinAverageOrientation = sin(averageOrientation);
    const double cosAverageOrientation = cos(averageOrientation);
    return
    {
        newX * cosAverageOrientation - newY * sinAverageOrientation,
        newX * sinAverageOrientation + newY * cosAverageOrientation,
        deltaAngle
    };
}
