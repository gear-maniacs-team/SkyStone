package org.firstinspires.ftc.teamcode.autonomous.demo

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import org.firstinspires.ftc.teamcode.TeamRobot
import org.firstinspires.ftc.teamcode.motors.Wheels
import org.firstinspires.ftc.teamcode.pid.PidController
import org.firstinspires.ftc.teamcode.sensors.Encoder
import kotlin.math.atan2
import kotlin.math.hypot
import kotlin.math.sin

abstract class EncoderAuto : LinearOpMode() {

    private val robot = TeamRobot()
    private val wheels = Wheels()
    private val encoder = Encoder()
    private val leftPid = PidController(64.0, 0.00001, 0.05).apply {
        setOutputRange(-1.0, 1.0)
    }
    private val backPid = PidController(64.0, 0.00001, 0.05).apply {
        setOutputRange(-1.0, 1.0)
    }

    final override fun runOpMode() {
        robot.init(hardwareMap, listOf(wheels, encoder), listOf(encoder))

        while (!isStarted) {
            telemetry.addData("Status", "Waiting for start")
            telemetry.update()
        }

        robot.start()

        onRun()

        robot.stop()
    }

    fun setTarget(targetX: Int, targetY: Int) {
        leftPid.setPoint = targetX.toDouble()
        backPid.setPoint = targetY.toDouble()

        var left: Double
        var back: Double

        do {
            left = encoder.left.currentPosition.toDouble()
            back = encoder.back.currentPosition.toDouble()
            Thread.sleep(100)

            telemetry.addData("Left Encoder", left)
            telemetry.addData("Back Encoder", back)

            val x = leftPid.compute(left)
            val y = backPid.compute(back)

            planeMovement(x, y)
        } while (left != 0.0 && back != 0.0)
    }

    private fun planeMovement(x: Double, y: Double) {
        val magnitude = hypot(x, y) * MOTOR_SPEED_MULTIPLIER
        val angle = atan2(y, x) - Math.PI / 2

        val speedX = magnitude * sin(angle + Math.PI / 4)
        val speedY = magnitude * sin(angle - Math.PI / 4)

        with(wheels) {
            rightFront.power = -speedX
            leftFront.power = -speedY
            rightBack.power = speedY
            leftBack.power = speedX
        }
    }

    abstract fun onRun()

    private companion object {
        private const val MOTOR_SPEED_MULTIPLIER = 0.7
    }
}

@Autonomous(name = "RedAutoEncoder")
class RedAutoEncoder : EncoderAuto() {
    override fun onRun() {
        setTarget(100, 100)
    }
}
