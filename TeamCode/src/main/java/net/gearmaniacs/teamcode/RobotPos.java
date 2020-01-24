package net.gearmaniacs.teamcode;

public final class RobotPos {

    public static volatile double currentX = 0.0;
    public static volatile double currentY = 0.0;
    public static volatile double currentAngle = 0.0; // Stored in Radians

    public static volatile double targetX = 0.0;
    public static volatile double targetY = 0.0;
    public static volatile double targetAngle = 0.0; // Stored in Radians

    public static void resetAll() {
        currentX = 0.0;
        currentY = 0.0;
        currentAngle = 0.0;

        targetX = 0.0;
        targetY = 0.0;
        targetAngle = 0.0;
    }
}
