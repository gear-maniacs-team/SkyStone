#pragma once

struct RobotPose
{
    double left{}, right{}, back{};
};

class Encoders
{
    RobotPose previousPose{};

public:
    /*
     * Resets the previousPose member
     */
    void init() noexcept;

    /*
     * Odometry Localization using Arcs
     *
     * @return Returns the change in the Robot Pose
     */
    RobotPose update(const RobotPose &pose, double currentAngle) noexcept;
};
