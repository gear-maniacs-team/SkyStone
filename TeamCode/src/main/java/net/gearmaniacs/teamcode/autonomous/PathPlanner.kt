package net.gearmaniacs.teamcode.autonomous

import android.annotation.SuppressLint
import android.util.Log
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import net.gearmaniacs.teamcode.RobotPos
import net.gearmaniacs.teamcode.TeamRobot
import net.gearmaniacs.teamcode.hardware.sensors.Encoders
import net.gearmaniacs.teamcode.teleop.MainTeleOp
import net.gearmaniacs.teamcode.utils.PathPoint
import kotlin.concurrent.thread

@TeleOp(name = "Path Planner")
class PathPlanner : MainTeleOp() {

    private val encoder = Encoders()
    private val list = ArrayList<PathPoint>()

    override fun init() {
        robot.init(
            hardwareMap,
            listOf(wheels, intake, lift, foundation, outtake, encoder),
            listOf(encoder)
        )
        super.init()
    }

    @SuppressLint("MissingSuperCall")
    override fun start() {
        robot.start()
    }

    override fun loop() {
        if (gamepad1.x) {
            val it = PathPoint(RobotPos.currentX, RobotPos.currentY, RobotPos.currentAngle)
            list.add(it)
            Log.v("Point", it.toString())
            Thread.sleep(400)
        }
        Thread.sleep(20)
    }
}