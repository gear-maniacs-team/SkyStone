package org.firstinspires.ftc.teamcode.pursuit

import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.RobotPos
import org.firstinspires.ftc.teamcode.TeamRobot
import org.firstinspires.ftc.teamcode.motors.Wheels
import org.firstinspires.ftc.teamcode.pid.PidController
import org.firstinspires.ftc.teamcode.sensors.Encoder
import kotlin.math.atan2
import kotlin.math.hypot
import kotlin.math.sin

@TeleOp(name = "Pursuit")
class PursuitOpMode : OpMode() {

    private val robot = TeamRobot()
    private val wheels = Wheels()
    private val encoder = Encoder()
    private val xPid = PidController(64.0, 0.0, 0.005)
    private val yPid = PidController(64.0, 0.0, 0.005)
    private val rotationPid = PidController(64.0, 0.0, 0.005)

    override fun init() {
        robot.init(hardwareMap, listOf(wheels, encoder), listOf(encoder))
        RobotPos.resetAll()

        xPid.setOutputRange(-0.7, 0.7)
        yPid.setOutputRange(-0.7, 0.7)
        rotationPid.setOutputRange(-0.4, 0.4)
    }

    override fun start() {
        robot.start()
    }

    override fun loop() {
        RobotMovement.goToPosition(20.0, 10.0, 1.0, Math.PI / 2, 0.3)

        xPid.setPoint = RobotPos.targetX
        yPid.setPoint = RobotPos.targetY
        rotationPid.setPoint = RobotPos.targetAngle

        val x = xPid.compute(RobotPos.currentX)
        val y = yPid.compute(RobotPos.currentY)
        val rotation = rotationPid.compute(RobotPos.currentAngle)

        movement(x, y, rotation)
    }

    override fun stop() {
        robot.stop()
    }

    private fun movement(x: Double, y: Double, rotationCorrection: Double) {
        val magnitude = hypot(x, y) * MOTOR_SPEED_MULTIPLIER
        val angle = atan2(y, x) - Math.PI / 2

        val speedX = magnitude * sin(angle + Math.PI / 4)
        val speedY = magnitude * sin(angle - Math.PI / 4)

        with(wheels) {
            rightFront.power = -speedX + rotationCorrection
            leftFront.power = -speedY + rotationCorrection
            rightBack.power = speedY + rotationCorrection
            leftBack.power = speedX + rotationCorrection
        }
    }

    companion object {
        const val MOTOR_SPEED_MULTIPLIER = 0.7
    }
}
