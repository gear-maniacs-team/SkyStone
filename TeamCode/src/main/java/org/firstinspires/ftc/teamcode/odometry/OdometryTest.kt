package org.firstinspires.ftc.teamcode.odometry

import com.qualcomm.robotcore.eventloop.opmode.Disabled
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.RobotPos
import org.firstinspires.ftc.teamcode.TeamRobot
import org.firstinspires.ftc.teamcode.motors.Wheels
import org.firstinspires.ftc.teamcode.sensors.Encoder
import kotlin.math.cos
import kotlin.math.sin

@Disabled
@TeleOp(name = "Odometry Test", group = "Odometry")
class OdometryTest : OpMode() {

    private val robot = TeamRobot()
    private val wheels = Wheels()
    private val encoder = Encoder()

    override fun init() {
        robot.init(hardwareMap, listOf(wheels, encoder), listOf(encoder))
    }

    override fun start() {
        robot.start()
    }

    override fun loop() {
        telemetry.addData("X Position", RobotPos.currentX)
        telemetry.addData("Y Position", RobotPos.currentY)
        telemetry.addData("Orientation", RobotPos.currentAngle)

        telemetry.addData("Left encoder position", encoder.left.currentPosition)
        telemetry.addData("Right encoder position", encoder.right.currentPosition)
        telemetry.addData("Back encoder position", encoder.back.currentPosition)

        telemetry.update()
    }

    override fun stop() {
        robot.stop()
    }

    /**
     * Calculate the power in the x direction
     * @param desiredAngle angle on the x axis
     * @param speed robot's speed
     * @return the x vector
     */
    private fun calculateX(desiredAngle: Double, speed: Double) = sin(Math.toRadians(desiredAngle)) * speed

    /**
     * Calculate the power in the y direction
     * @param desiredAngle angle on the y axis
     * @param speed robot's speed
     * @return the y vector
     */
    private fun calculateY(desiredAngle: Double, speed: Double) = cos(Math.toRadians(desiredAngle)) * speed
}