package org.firstinspires.ftc.teamcode.teleop

import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.Servo
import org.firstinspires.ftc.teamcode.TeamRobot
import org.firstinspires.ftc.teamcode.utils.fastLazy

@TeleOp(name = "QubeTeleOp")
class QubeTeleOp : OpMode() {

    private val robot = TeamRobot()
    private val wheelMotors by fastLazy { robot.wheelsMotors }
    private lateinit var leftIntake: DcMotor
    private lateinit var rightIntake: DcMotor
    private lateinit var cargoServo: Servo
    private lateinit var leftExtension: Servo
    private lateinit var rightExtension: Servo
    private var precisionModeOn = false

    private val velocityFront = getVelocityForRpmAndEncoderCycles(220, 383.6)
    private val velocityBack = getVelocityForRpmAndEncoderCycles(220, 753.2)

    override fun init() {
        robot.init(hardwareMap)

        wheelMotors.setModeAll(DcMotor.RunMode.RUN_USING_ENCODER)

        leftIntake = hardwareMap.dcMotor["intake_left"]
        rightIntake = hardwareMap.dcMotor["intake_right"]

        cargoServo = hardwareMap.servo["iesi_afara"]
        leftExtension = hardwareMap.servo["extension_left"]
        rightExtension = hardwareMap.servo["extension_right"]
    }

    override fun start() {
        Thread {
            while (robot.isOpModeActive) {
                intake()
                cargo()
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

        var powerLeft = y + x
        var powerRight = -y + x

        val max = maxOf(powerLeft, powerRight)

        if (max > 1) {
            powerLeft /= max
            powerRight /= max
        }

        precisionModeOn = gamepad1.a

        if (precisionModeOn) {
            powerRight *= PRECISION_MODE_MULTIPLIER
            powerLeft *= PRECISION_MODE_MULTIPLIER
        }

        with(wheelMotors) {
            rightFront.velocity = velocityFront * MOTOR_SPEED_MULTIPLIER
            leftFront.velocity = velocityFront * MOTOR_SPEED_MULTIPLIER
            rightBack.velocity = velocityBack * MOTOR_SPEED_MULTIPLIER
            leftBack.velocity = velocityBack * MOTOR_SPEED_MULTIPLIER
        }
    }

    private fun strafe() {
        with(wheelMotors) {
            // Strafe Right
            while (gamepad1.right_stick_x > 0) {
                rightFront.velocity = -velocityFront * MOTOR_SPEED_STRAFE
                leftFront.velocity = -velocityFront * MOTOR_SPEED_STRAFE
                rightBack.velocity = velocityBack * MOTOR_SPEED_STRAFE
                leftBack.velocity = velocityBack * MOTOR_SPEED_STRAFE
            }

            // Strafe Left
            while (gamepad1.right_stick_x < 0) {
                rightFront.velocity = velocityFront * MOTOR_SPEED_STRAFE
                leftFront.velocity = velocityFront * MOTOR_SPEED_STRAFE
                rightBack.velocity = -velocityBack * MOTOR_SPEED_STRAFE
                leftBack.velocity = -velocityBack * MOTOR_SPEED_STRAFE
            }
        }
    }

    private fun intake() {
        if (gamepad2.dpad_left) {
            leftIntake.power = INTAKE_POWER
            return
        } else if (gamepad2.dpad_right) {
            rightIntake.power = -INTAKE_POWER * 1.333
            return
        }

        val intakePower = when {
            gamepad2.right_bumper -> INTAKE_POWER
            gamepad2.left_bumper -> -INTAKE_POWER
            gamepad2.dpad_down -> INTAKE_POWER / 2
            gamepad2.dpad_up-> -INTAKE_POWER / 2
            else -> 0.0
        }

        leftIntake.power = intakePower
        rightIntake.power = -intakePower * 1.333
    }

    private fun cargo() {
        if (gamepad2.x)
            cargoServo.position = 0.1
        else if (gamepad2.a)
            cargoServo.position = 0.6
    }

    private fun extension() {
        if (gamepad2.y) {
            leftExtension.position = 0.0
            rightExtension.position = 0.0
        } else if (gamepad2.b) {
            leftExtension.position = 1.0
            rightExtension.position = 1.0
        }
    }

    private fun getVelocityForRpmAndEncoderCycles(rpm: Int, encoder: Double) = rpm * (encoder / 60)

    private companion object {
        private const val PRECISION_MODE_MULTIPLIER = 0.45
        private const val MOTOR_SPEED_MULTIPLIER = 0.9
        private const val MOTOR_SPEED_STRAFE = 0.6
        private const val INTAKE_POWER = 0.75
    }
}
