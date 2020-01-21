package org.firstinspires.ftc.teamcode.odometry

import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.RobotPos
import org.firstinspires.ftc.teamcode.TeamRobot
import org.firstinspires.ftc.teamcode.motors.Wheels
import org.firstinspires.ftc.teamcode.sensors.Encoder
import org.firstinspires.ftc.teamcode.utils.MathUtils.angleWrap
import kotlin.math.cos
import kotlin.math.sin

@TeleOp(name = "Odometry Test", group = "Odometry")
class OdometryTest : OpMode() {

    private val robot = TeamRobot()
    private val encoder = Encoder()

    override fun init() {
        robot.init(hardwareMap, listOf(encoder), listOf(encoder))
        RobotPos.resetAll()

        encoder.useBulkRead = true
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
