package org.firstinspires.ftc.teamcode.autonomous.demo

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.hardware.DcMotor
import org.firstinspires.ftc.teamcode.RobotPos
import org.firstinspires.ftc.teamcode.TeamRobot
import org.firstinspires.ftc.teamcode.motors.Wheels
import org.firstinspires.ftc.teamcode.pid.PidController
import org.firstinspires.ftc.teamcode.utils.getDevice
import kotlin.math.*

abstract class EncoderAuto : LinearOpMode() {

    private val robot = TeamRobot()
    private val wheels = Wheels()
    private val leftPid = PidController(64.0, 0.00001, 0.05).apply {
        setOutputRange(-1.0, 1.0)
    }
    private val backPid = PidController(64.0, 0.00001, 0.05).apply {
        setOutputRange(-1.0, 1.0)
    }
    private val strafePid = PidController(256.0, 0.0001, 0.5).apply {
        setOutputRange(-100.0, 100.0)
    }
    private lateinit var encoderLeft: DcMotor
    private lateinit var encoderBack: DcMotor
    private var resetAngle = 0.0

    final override fun runOpMode() {
        robot.init(hardwareMap, listOf(wheels))
        encoderLeft = hardwareMap.getDevice("encoder_left")
        encoderBack = hardwareMap.getDevice("encoder_back")

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
            left = encoderLeft.currentPosition.toDouble()
            back = encoderBack.currentPosition.toDouble()
            Thread.sleep(100)

            telemetry.addData("Left Encoder", left)
            telemetry.addData("Back Encoder", back)

            val x = leftPid.compute(left)
            val y = backPid.compute(back)

            planeMovement(x, y)
        } while (left != 0.0 && back != 0.0)
    }

    private fun planeMovement(x: Double, y: Double) {
//        if (resetStrafePid && !curvedMovement) {
//            strafePid.reset()
//            strafePid.setPoint = RobotPos.currentAngle
//            resetStrafePid = false
//        }

        val correction = strafePid.compute(RobotPos.currentAngle)
        val frontCorrection = Wheels.rpmToTps(correction, FRONT_ENCODER_COUNT)
        val backCorrection = Wheels.rpmToTps(correction, BACK_ENCODER_COUNT)

        val magnitude = hypot(x, y) * MOTOR_SPEED_MULTIPLIER

        val independentAngleCorrection = RobotPos.currentAngle - resetAngle
        val angle = atan2(y, x) - Math.PI / 2 - independentAngleCorrection

        val speedX = magnitude * sin(angle + Math.PI / 4)
        val speedY = magnitude * sin(angle - Math.PI / 4)

        with(wheels) {
            rightFront.velocity = min(-getFrontVelocity(speedX) + frontCorrection, MAX_FRONT_VELOCITY)
            leftFront.velocity = min(-getFrontVelocity(speedY) + frontCorrection, MAX_FRONT_VELOCITY)
            rightBack.velocity = min(getBackVelocity(speedY) + backCorrection, MAX_BACK_VELOCITY)
            leftBack.velocity = min(getBackVelocity(speedX) + backCorrection, MAX_BACK_VELOCITY)
        }

        with(telemetry) {
            addData("Strafe Correction", correction)
            update()
        }
    }

    abstract fun onRun()

    private companion object {
        private const val MOTOR_SPEED_MULTIPLIER = 0.7
        private const val MAX_RPM = 223.0
        private const val FRONT_ENCODER_COUNT = 753.2
        private const val BACK_ENCODER_COUNT = 383.6

        private fun getFrontVelocity(power: Double) =
                Wheels.rpmToTps(MAX_RPM * power, FRONT_ENCODER_COUNT)

        private fun getBackVelocity(power: Double) =
                Wheels.rpmToTps(MAX_RPM * power, BACK_ENCODER_COUNT)

        private val MAX_FRONT_VELOCITY = getFrontVelocity(1.0)
        private val MAX_BACK_VELOCITY = getBackVelocity(1.0)
    }
}

@Autonomous(name = "RedAutoEncoder")
class RedAutoEncoder : EncoderAuto() {
    override fun onRun() {
        setTarget(100, 100)
    }
}
