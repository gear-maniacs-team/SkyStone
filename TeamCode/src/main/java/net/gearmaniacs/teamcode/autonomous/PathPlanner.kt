package net.gearmaniacs.teamcode.autonomous

import android.util.Log
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import net.gearmaniacs.teamcode.RobotPos
import net.gearmaniacs.teamcode.teleop.odometry.OdometryTest
import net.gearmaniacs.teamcode.utils.PathPoint

@TeleOp(name = "Path Planner")
class PathPlanner : OdometryTest() {

    override fun loop() {
        super.loop()

        if (gamepad1.x) {
            val it = PathPoint(RobotPos.currentX, RobotPos.currentY, RobotPos.currentAngle)
            Log.v("Point", it.toString())
            Thread.sleep(400)
        }
    }
}
