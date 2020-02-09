package net.gearmaniacs.teamcode.teleop.odometry

import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import net.gearmaniacs.teamcode.RobotPos
import net.gearmaniacs.teamcode.TeamOpMode
import net.gearmaniacs.teamcode.hardware.sensors.Encoders
import net.gearmaniacs.teamcode.utils.MathUtils.angleWrap
import net.gearmaniacs.teamcode.utils.PerformanceProfiler
import net.gearmaniacs.teamcode.utils.getCurrentPosition

@TeleOp(name = "Odometry Test", group = "Odometry")
class OdometryTest : TeamOpMode() {

    private val encoder = Encoders()
    private val performanceProfiler = PerformanceProfiler()

    override fun init() {
        robot.init(hardwareMap, listOf(encoder), listOf(encoder))
        RobotPos.resetAll()
    }

    override fun loop() {
        performanceProfiler.update(telemetry)

        telemetry.addData("X Position", "%.3f", RobotPos.currentX)
        telemetry.addData("Y Position", "%.3f", RobotPos.currentY)
        telemetry.addData("Radians", "%.3f", angleWrap(RobotPos.currentAngle))
        telemetry.addData("--", "--")

        val leftPos = -encoder.left.getCurrentPosition(robot.bulkData2).toDouble()
        val rightPos = encoder.right.getCurrentPosition(robot.bulkData1).toDouble()
        val backPos = encoder.back.getCurrentPosition(robot.bulkData2).toDouble()

        telemetry.addData("Left encoder", leftPos)
        telemetry.addData("Right encoder", rightPos)
        telemetry.addData("Back encoder", backPos)
    }
}
