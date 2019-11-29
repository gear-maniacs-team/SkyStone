package org.firstinspires.ftc.teamcode.teleop

import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.Servo
import org.firstinspires.ftc.teamcode.TeamRobot
import org.firstinspires.ftc.teamcode.utils.fastLazy
import org.firstinspires.ftc.teamcode.utils.getVelocityForRpmAndEncoderCycles

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

    private val maxFrontVelocity = getVelocityForRpmAndEncoderCycles(223.0, 383.6)
    private val maxBackVelocity = getVelocityForRpmAndEncoderCycles(223.0, 753.2)

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
                //extension()
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

        powerRight *= MOTOR_SPEED_MULTIPLIER
        powerLeft *= MOTOR_SPEED_MULTIPLIER

        precisionModeOn = gamepad1.a

        if (precisionModeOn) {
            powerRight *= PRECISION_MODE_MULTIPLIER
            powerLeft *= PRECISION_MODE_MULTIPLIER
        }

        with(wheelMotors) {
            rightFront.velocity = maxFrontVelocity * powerRight
            leftFront.velocity = maxFrontVelocity * powerLeft
            rightBack.velocity = maxBackVelocity * powerRight
            leftBack.velocity = maxBackVelocity * powerLeft
        }
    }

    private fun strafe() {
        with(wheelMotors) {
            // Strafe Right
            while (gamepad1.right_stick_x > 0) {
                rightFront.velocity = -maxFrontVelocity * MOTOR_SPEED_STRAFE
                leftFront.velocity = -maxFrontVelocity * MOTOR_SPEED_STRAFE
                rightBack.velocity = maxBackVelocity * MOTOR_SPEED_STRAFE
                leftBack.velocity = maxBackVelocity * MOTOR_SPEED_STRAFE
            }

            // Strafe Left
            while (gamepad1.right_stick_x < 0) {
                rightFront.velocity = maxFrontVelocity * MOTOR_SPEED_STRAFE
                leftFront.velocity = maxFrontVelocity * MOTOR_SPEED_STRAFE
                rightBack.velocity = -maxBackVelocity * MOTOR_SPEED_STRAFE
                leftBack.velocity = -maxBackVelocity * MOTOR_SPEED_STRAFE
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
        cargoServo.position = if (gamepad2.a) 1.0 else 0.0
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

    private companion object {
        private const val PRECISION_MODE_MULTIPLIER = 0.45
        private const val MOTOR_SPEED_MULTIPLIER = 0.9
        private const val MOTOR_SPEED_STRAFE = 0.6
        private const val INTAKE_POWER = 0.75
    }
}
