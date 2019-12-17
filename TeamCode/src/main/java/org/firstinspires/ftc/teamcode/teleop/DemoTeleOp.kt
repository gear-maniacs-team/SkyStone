package org.firstinspires.ftc.teamcode.teleop

import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.Servo
import org.firstinspires.ftc.teamcode.TeamRobot
import org.firstinspires.ftc.teamcode.motors.Intake
import org.firstinspires.ftc.teamcode.motors.Wheels
import org.firstinspires.ftc.teamcode.utils.rpmToTps
import kotlin.concurrent.thread
import kotlin.math.*

@TeleOp(name = "DemoTeleOp")
class DemoTeleOp : OpMode() {

    private val robot = TeamRobot()
    private val wheels = Wheels()
    private val intake = Intake()

    private var releaseServo: Servo? = null
    private var cargoServo: Servo? = null

    private val maxFrontVelocity = getFrontVelocity(1.0)
    private val maxBackVelocity = getBackVelocity(1.0)

    private var precisionModeOn = false
    private var rightFrontVelocity = 0.0
    private var leftFrontVelocity = 0.0
    private var rightBackVelocity = 0.0
    private var leftBackVelocity = 0.0

    override fun init() {
        robot.init(hardwareMap, listOf(wheels, intake))
        wheels.setModeAll(DcMotor.RunMode.RUN_USING_ENCODER)

        releaseServo = hardwareMap.servo["front"]
        cargoServo = hardwareMap.servo["cargo"]
    }

    override fun start() {
        robot.start()

        releaseServo?.position = 0.0

        thread {
            while (robot.isOpModeActive) {
                intake()
                cargo()
            }
        }
    }

    override fun loop() {
        precisionModeOn = gamepad1.right_bumper

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

        var speedLeft = y + x
        var speedRight = -y + x

        val max = maxOf(speedLeft, speedRight)

        if (max > 1) {
            speedLeft /= max
            speedRight /= max
        }

        if (precisionModeOn) {
            speedRight *= PRECISION_MODE_MULTIPLIER
            speedLeft *= PRECISION_MODE_MULTIPLIER
        } else {
            speedRight *= MOTOR_SPEED_MULTIPLIER
            speedLeft *= MOTOR_SPEED_MULTIPLIER
        }

        rightFrontVelocity += getFrontVelocity(speedRight)
        leftFrontVelocity += getFrontVelocity(speedLeft)
        rightBackVelocity += getBackVelocity(speedRight)
        leftBackVelocity += getBackVelocity(speedLeft)
    }

    private fun planeMovement() {
        val x = gamepad1.right_stick_x.toDouble()
        val y = -gamepad1.right_stick_y.toDouble()

        if (abs(x) == 0.0 && abs(y) == 0.0)
            return

        var magnitude = hypot(x, y)
        magnitude *= if (precisionModeOn) PRECISION_MODE_MULTIPLIER else MOTOR_SPEED_MULTIPLIER

        val angle = atan2(x, y) - Math.PI / 4

        val speed1 = magnitude * cos(angle)
        val speed2 = magnitude * sin(angle)

        rightFrontVelocity -= getFrontVelocity(speed1)
        leftFrontVelocity -= getFrontVelocity(speed2)
        rightBackVelocity += getBackVelocity(speed1)
        leftBackVelocity += getBackVelocity(speed2)
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
        rpmToTps(MAX_RPM * power, FRONT_ENCODER_COUNT)

    private fun getBackVelocity(power: Double) =
        rpmToTps(MAX_RPM * power, BACK_ENCODER_COUNT)

    private companion object {
        private const val MAX_RPM = 223.0
        private const val FRONT_ENCODER_COUNT = 383.6
        private const val BACK_ENCODER_COUNT = 753.2

        private const val PRECISION_MODE_MULTIPLIER = 0.45
        private const val MOTOR_SPEED_MULTIPLIER = 0.85
        private const val INTAKE_POWER = 0.6
    }
}
