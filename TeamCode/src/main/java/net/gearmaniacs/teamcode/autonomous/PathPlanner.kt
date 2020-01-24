package net.gearmaniacs.teamcode.autonomous

import android.util.Log
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import net.gearmaniacs.teamcode.RobotPos
import net.gearmaniacs.teamcode.TeamRobot
import net.gearmaniacs.teamcode.teleop.NewTeleOp
import net.gearmaniacs.teamcode.utils.PathPoint
import kotlin.concurrent.thread

@TeleOp(name = "Path Planner")
class PathPlanner : NewTeleOp() {

    private val list = ArrayList<PathPoint>()

    override fun start() {
        super.start()
        thread {
            while (TeamRobot.getRobot().isOpModeActive) {
                if (gamepad1.x) {
                    val it = PathPoint(RobotPos.currentX, RobotPos.currentY, RobotPos.currentAngle)
                    list.add(it)
                    Log.v("Point", it.toString())
                    Thread.sleep(200)
                }
                Thread.sleep(20)
            }
        }
    }
}