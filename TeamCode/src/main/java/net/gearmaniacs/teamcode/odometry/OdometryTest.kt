package net.gearmaniacs.teamcode.odometry

import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import net.gearmaniacs.teamcode.RobotPos
import net.gearmaniacs.teamcode.TeamRobot
import net.gearmaniacs.teamcode.hardware.sensors.Encoders
import net.gearmaniacs.teamcode.utils.MathUtils.angleWrap
import net.gearmaniacs.teamcode.utils.PerformanceProfiler

@TeleOp(name = "Odometry Test", group = "Odometry")
class
OdometryTest : OpMode() {

    private val robot = TeamRobot()
    private val encoder = Encoders()
    private val performanceProfiler = PerformanceProfiler()

    override fun init() {
        robot.useBulkRead = false
        robot.init(hardwareMap, listOf(encoder), listOf(encoder))
        RobotPos.resetAll()
    }

    override fun start() {
        robot.start()
    }

    override fun loop() {
        performanceProfiler.update(telemetry)

        telemetry.addData("X Position", "%.3f", RobotPos.currentX)
        telemetry.addData("Y Position", "%.3f", RobotPos.currentY)
        telemetry.addData("Radians", "%.3f", angleWrap(RobotPos.currentAngle))
        telemetry.addData("--", "--")

        val leftPos: Int
        val rightPos: Int
        val backPos: Int

        if (robot.useBulkRead) {
            leftPos = -robot.bulkData2.getMotorCurrentPosition(encoder.left)
            rightPos = robot.bulkData1.getMotorCurrentPosition(encoder.right)
            backPos = robot.bulkData2.getMotorCurrentPosition(encoder.back)
        } else {
            leftPos = -encoder.left.currentPosition
            rightPos = encoder.right.currentPosition
            backPos = encoder.back.currentPosition
        }

        telemetry.addData("Left encoder", leftPos)
        telemetry.addData("Right encoder", rightPos)
        telemetry.addData("Back encoder", backPos)
    }

    override fun stop() {
        robot.stop()
    }
}
