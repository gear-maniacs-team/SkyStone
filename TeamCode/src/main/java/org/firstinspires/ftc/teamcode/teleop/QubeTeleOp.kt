package org.firstinspires.ftc.teamcode.teleop

import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.Servo
import org.firstinspires.ftc.teamcode.QubeRobot
import org.firstinspires.ftc.teamcode.utils.fastLazy

@TeleOp(name = "QubeTeleOp")
class QubeTeleOp : OpMode() {

    private val robot = QubeRobot()
    private val wheelMotors by fastLazy { robot.wheelsMotors }
    private lateinit var leftIntake: DcMotor
    private lateinit var rightIntake: DcMotor
    private lateinit var poopOut: Servo
    private lateinit var leftExtension: Servo
    private lateinit var rightExtension: Servo
    private var precisionModeOn = false

    override fun init() {
        robot.init(hardwareMap)

        wheelMotors.setModeAll(DcMotor.RunMode.RUN_USING_ENCODER)
        leftIntake = hardwareMap.dcMotor["intake_left"]
        rightIntake = hardwareMap.dcMotor["intake_right"]

        poopOut = hardwareMap.servo["iesi_afara"]
        leftExtension = hardwareMap.servo["extension_left"]
        rightExtension = hardwareMap.servo["extension_right"]
    }

    override fun start() {
        Thread {
            while (robot.isOpModeActive) {
                intake()
                getTheFuckOut()
                extension()
            }
        }.start()
    }

    override fun loop() {
        movement()
        strafe()

        telemetry.addData("Precision Mode On", "%b\n", precisionModeOn)
        telemetry.update()
    }

    override fun stop() {
        robot.stop()
    }

    private fun movement() {
        val x = (-gamepad1.left_stick_x).toDouble()
        val y = (gamepad1.left_stick_y).toDouble()

        var powerLeftBack = y + x
        var powerLeftFront = y + x
        var powerRightFront = -y + x
        var powerRightBack = -y + x

        // Find the biggest value
        val max = maxOf(maxOf(powerLeftBack, powerLeftFront, powerRightFront), powerRightBack)

        if (max > 1) {
            powerLeftFront /= max
            powerRightFront /= max
            powerLeftBack /= max
            powerRightBack /= max
        }

        precisionModeOn = gamepad1.a

        if (precisionModeOn) {
            powerRightFront *= PRECISION_MODE_MULTIPLIER
            powerLeftFront *= PRECISION_MODE_MULTIPLIER
            powerRightBack *= PRECISION_MODE_MULTIPLIER
            powerLeftBack *= PRECISION_MODE_MULTIPLIER
        }

        with(wheelMotors) {
            // Front motors have double the rpm :(
            rightFront.power = powerRightFront * MOTOR_SPEED_MULTIPLIER / 2
            leftFront.power = powerLeftFront * MOTOR_SPEED_MULTIPLIER / 2
            rightBack.power = powerRightBack * MOTOR_SPEED_MULTIPLIER
            leftBack.power = powerLeftBack * MOTOR_SPEED_MULTIPLIER
        }
    }

    private fun strafe() {
        // Strafe Right
        while (gamepad1.right_stick_x > 0) {
            wheelMotors.rightFront.power = -MOTOR_SPEED_STRAFE / 2
            wheelMotors.leftFront.power = -MOTOR_SPEED_STRAFE / 2
            wheelMotors.rightBack.power = MOTOR_SPEED_STRAFE
            wheelMotors.leftBack.power = MOTOR_SPEED_STRAFE
        }

        // Strafe Left
        while (gamepad1.right_stick_x < 0) {
            wheelMotors.rightFront.power = MOTOR_SPEED_STRAFE / 2
            wheelMotors.leftFront.power = MOTOR_SPEED_STRAFE / 2
            wheelMotors.rightBack.power = -MOTOR_SPEED_STRAFE
            wheelMotors.leftBack.power = -MOTOR_SPEED_STRAFE
        }
    }

    private fun intake() {
        val intakePower = when {
            gamepad2.right_bumper -> INTAKE_POWER
            gamepad2.left_bumper -> -INTAKE_POWER
            else -> 0.0
        }

        leftIntake.power = intakePower
        rightIntake.power = -intakePower
    }

    private fun getTheFuckOut() {
        if (gamepad2.x)
            poopOut.position = 0.0
        else if (gamepad2.a)
            poopOut.position = 1.0
    }

    private fun extension() {
        if (gamepad2.y) {
            leftExtension.position = 0.0
            rightExtension.position = 0.0
        } else if (gamepad2.b) {
            leftExtension.position = 1.0
            rightExtension.position = -1.0
        }
    }

    private companion object {
        private const val PRECISION_MODE_MULTIPLIER = 0.45
        private const val MOTOR_SPEED_MULTIPLIER = 0.8
        private const val MOTOR_SPEED_STRAFE = 0.6
        private const val INTAKE_POWER = 0.4
    }
}
