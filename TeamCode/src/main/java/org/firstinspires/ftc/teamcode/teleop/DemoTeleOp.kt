package org.firstinspires.ftc.teamcode.teleop

import android.util.Log
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.Servo
import org.firstinspires.ftc.teamcode.RobotPos
import org.firstinspires.ftc.teamcode.TeamRobot
import org.firstinspires.ftc.teamcode.motors.Intake
import org.firstinspires.ftc.teamcode.motors.Wheels
import org.firstinspires.ftc.teamcode.pid.PidController
import org.firstinspires.ftc.teamcode.sensors.Gyro
import kotlin.concurrent.thread
import kotlin.math.*

@TeleOp(name = "DemoTeleOp")
class DemoTeleOp : OpMode() {

    private val robot = TeamRobot()
    private val wheels = Wheels()
    private val intake = Intake()
    private val gyro = Gyro()

    private val strafePid = PidController(500.6, 0.0, 0.0).apply {
        tolerance = 0.000001
        setOutputRange(-100.0, 100.0)
    }

    private var releaseServo: Servo? = null
    private var cargoServo: Servo? = null

    private val maxFrontVelocity = getFrontVelocity(1.0)
    private val maxBackVelocity = getBackVelocity(1.0)

    private var precisionModeOn = false
    private var curvedMovement = false
    private var rightFrontVelocity = 0.0
    private var leftFrontVelocity = 0.0
    private var rightBackVelocity = 0.0
    private var leftBackVelocity = 0.0
    private var resetStrafePid = true

    override fun init() {
        robot.init(hardwareMap, listOf(wheels, intake, gyro), listOf(gyro))

        wheels.setModeAll(DcMotor.RunMode.RUN_USING_ENCODER)
        wheels.setZeroPowerBehaviorAll(DcMotor.ZeroPowerBehavior.BRAKE)

        releaseServo = hardwareMap.servo["front"]
        cargoServo = hardwareMap.servo["cargo"]
    }

    override fun start() {
        robot.start()

        //releaseServo?.position = 0.0

        thread {
            while (robot.isOpModeActive) {
                intake()
                cargo()
                Thread.yield()
            }
        }
    }

    override fun loop() {
        precisionModeOn = gamepad1.right_bumper

        curvedMovement = false
        rightFrontVelocity = 0.0
        leftFrontVelocity = 0.0
        rightBackVelocity = 0.0
        leftBackVelocity = 0.0

        curveMovement()
        planeMovement()

        with(wheels) {
            rightFront.velocity = min(rightFrontVelocity, maxFrontVelocity)
            leftFront.velocity = min(leftFrontVelocity, maxFrontVelocity)
            rightBack.velocity = min(rightBackVelocity, maxBackVelocity)
            leftBack.velocity = min(leftBackVelocity, maxBackVelocity)
        }

        with(telemetry) {
            addData("Current Angle", RobotPos.currentAngle)
            addData("Target Angle", strafePid.setPoint)

            addData("Precision Mode On", "%b\n", precisionModeOn)
            addData("Front Left Velocity", leftFrontVelocity)
            addData("Front Right Velocity", rightFrontVelocity)
            addData("Back Left Velocity", leftBackVelocity)
            addData("Back Right Velocity", rightBackVelocity)
            update()
        }
    }

    override fun stop() {
        robot.stop()
    }

    private fun curveMovement() {
        val x = -gamepad1.left_stick_x.toDouble()
        val y = gamepad1.left_stick_y.toDouble()

        if (abs(x) == 0.0 && abs(y) == 0.0) {
            return
        }

        var speedLeft = y + x
        var speedRight = -y + x

        val max = maxOf(speedLeft, speedRight)

        if (max > 1) {
            speedLeft /= max
            speedRight /= max
        }

        val percentage = if (precisionModeOn) PRECISION_MODE_MULTIPLIER else MOTOR_SPEED_MULTIPLIER

        speedRight *= percentage
        speedLeft *= percentage

        curvedMovement = true
        rightFrontVelocity += getFrontVelocity(speedRight)
        leftFrontVelocity += getFrontVelocity(speedLeft)
        rightBackVelocity += getBackVelocity(speedRight)
        leftBackVelocity += getBackVelocity(speedLeft)
    }

    private fun planeMovement() {
        val x = -gamepad1.right_stick_x.toDouble()
        val y = gamepad1.right_stick_y.toDouble()

        if (abs(x) == 0.0 && abs(y) == 0.0) {
            resetStrafePid = true
            return
        }

        if (resetStrafePid && !curvedMovement) {
            strafePid.reset()
            strafePid.setPoint = RobotPos.currentAngle
            resetStrafePid = false
        }

        val correction = if (!curvedMovement) strafePid.compute(RobotPos.currentAngle) else 0.0
        val frontCorrection = Wheels.rpmToTps(correction, FRONT_ENCODER_COUNT)
        val backCorrection = Wheels.rpmToTps(correction, BACK_ENCODER_COUNT)

        val magnitude = hypot(x, y) * if (precisionModeOn) PRECISION_MODE_MULTIPLIER else MOTOR_SPEED_MULTIPLIER

        val angle = atan2(y, x) - Math.PI / 2

        val speedX = magnitude * sin(angle + Math.PI / 4)
        val speedY = magnitude * sin(angle - Math.PI / 4)

        rightFrontVelocity += -getFrontVelocity(speedX) + frontCorrection
        leftFrontVelocity += -getFrontVelocity(speedY) + frontCorrection
        rightBackVelocity += getBackVelocity(speedY) + backCorrection
        leftBackVelocity += getBackVelocity(speedX) + backCorrection

        Log.d("PLANE", "Angle $angle X $x Y $y")
        telemetry.addData("Strafe Correction", correction)
    }

    private fun intake() {
        if (gamepad2.dpad_left) {
            intake.left.power = INTAKE_POWER
            return
        } else if (gamepad2.dpad_right) {
            intake.right.power = -INTAKE_POWER
            return
        }

        val intakePower = when {
            gamepad2.right_bumper -> INTAKE_POWER
            gamepad2.left_bumper -> -INTAKE_POWER * 0.8
            gamepad2.dpad_down -> INTAKE_POWER / 2
            gamepad2.dpad_up -> -INTAKE_POWER / 2
            else -> 0.0
        }

        intake.left.power = intakePower
        intake.right.power = -intakePower
    }

    private fun cargo() {
        cargoServo?.position = if (gamepad2.a) 1.0 else 0.0
    }

    /*private fun extension() {
        if (gamepad2.y) {
            leftExtension.position = 0.0
            rightExtension.position = 0.0
        } else if (gamepad2.b) {
            leftExtension.position = 1.0
            rightExtension.position = 1.0
        }
    }*/

    private fun getFrontVelocity(power: Double) =
        Wheels.rpmToTps(MAX_RPM * power, FRONT_ENCODER_COUNT)

    private fun getBackVelocity(power: Double) =
        Wheels.rpmToTps(MAX_RPM * power, BACK_ENCODER_COUNT)

    private companion object {
        private const val MAX_RPM = 223.0
        private const val FRONT_ENCODER_COUNT = 383.6
        private const val BACK_ENCODER_COUNT = 753.2

        private const val PRECISION_MODE_MULTIPLIER = 0.45
        private const val MOTOR_SPEED_MULTIPLIER = 0.9
        private const val INTAKE_POWER = 0.6
    }
}
