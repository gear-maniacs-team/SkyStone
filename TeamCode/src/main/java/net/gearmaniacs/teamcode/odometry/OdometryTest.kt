package net.gearmaniacs.teamcode.odometry

import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import net.gearmaniacs.teamcode.RobotPos
import net.gearmaniacs.teamcode.TeamRobot
import net.gearmaniacs.teamcode.sensors.Encoder
import net.gearmaniacs.teamcode.utils.MathUtils.angleWrap

@TeleOp(name = "Odometry Test", group = "Odometry")
class OdometryTest : OpMode() {

    private val robot = TeamRobot()
    private val encoder = Encoder()

    override fun init() {
        robot.init(hardwareMap, listOf(encoder), listOf(encoder))
        RobotPos.resetAll()

        encoder.useBulkRead = false
    }

    override fun start() {
        robot.start()
    }

    override fun loop() {
        robot.updateExpansionHubs()
        encoder.update()

        telemetry.addData("X Position", RobotPos.currentX)
        telemetry.addData("Y Position", RobotPos.currentY)
        telemetry.addData("Orientation", angleWrap(RobotPos.currentAngle))
        telemetry.addData("--", "--")

        if (encoder.useBulkRead) {
            telemetry.addData("Left encoder", -robot.bulkInputData2.getMotorCurrentPosition(encoder.left))
            telemetry.addData("Right encoder", robot.bulkInputData1.getMotorCurrentPosition(encoder.right))
            telemetry.addData("Back encoder", robot.bulkInputData1.getMotorCurrentPosition(encoder.back))
        } else {
            telemetry.addData("Left encoder", -encoder.left.currentPosition)
            telemetry.addData("Right encoder", encoder.right.currentPosition)
            telemetry.addData("Back encoder", encoder.back.currentPosition)
        }

        telemetry.update()
    }

    override fun stop() {
        robot.stop()
    }
}
