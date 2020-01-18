package org.firstinspires.ftc.teamcode.odometry

import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.RobotPos
import org.firstinspires.ftc.teamcode.sensors.Encoder
import org.firstinspires.ftc.teamcode.utils.MathUtils.angleWrap
import kotlin.math.cos
import kotlin.math.sin

@TeleOp(name = "Odometry Test", group = "Odometry")
class OdometryTest : OpMode() {

    //    private val robot = TeamRobot()
//    private val wheels = Wheels()
    private val encoder = Encoder()

    override fun init() {
        encoder.init(hardwareMap)
        RobotPos.resetAll()
//        robot.init(hardwareMap, listOf(wheels, encoder), listOf(encoder))
    }

    override fun start() {
//        robot.start()
    }

    override fun loop() {
        encoder.update()

        telemetry.addData("X Position", Encoder.ticksToCM(RobotPos.currentX))
        telemetry.addData("Y Position", Encoder.ticksToCM(RobotPos.currentY))
        telemetry.addData("Orientation", angleWrap(RobotPos.currentAngle))

        telemetry.addData("Left encoder position", encoder.left.currentPosition)
        telemetry.addData("Right encoder position", encoder.right.currentPosition)
        telemetry.addData("Back encoder position", encoder.back.currentPosition)

        telemetry.update()
    }

    override fun stop() {
//        robot.stop()
    }
}
