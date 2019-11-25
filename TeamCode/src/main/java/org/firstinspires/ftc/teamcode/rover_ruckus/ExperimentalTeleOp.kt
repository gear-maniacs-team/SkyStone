package org.firstinspires.ftc.teamcode.rover_ruckus

import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DcMotor
import org.firstinspires.ftc.teamcode.utils.fastLazy

@TeleOp(name = "ExperimentalTeleOp")
class ExperimentalTeleOp : OpMode() {

    private val robot = RoverRuckusRobot()
    private val wheelMotors by fastLazy { robot.wheelsMotors }
    private val armMotors by fastLazy { robot.armMotors }
    private var precisionModeOn = false

    override fun init() {
        robot.init(hardwareMap)

        wheelMotors.setModeAll(DcMotor.RunMode.RUN_USING_ENCODER)
    }

    override fun start() {
        Thread {
            while (robot.isOpModeActive) {
                armMovement()
                collector()
            }
        }.start()
    }

    override fun loop() {
        movement()
        strafe()
        latching()

        telemetry.addData("Precision Mode On", "%b\n", precisionModeOn)
        telemetry.addData("Latching Power", armMotors.latchMotor.power)
        telemetry.addData("Arm Angle Power", armMotors.armAngle.power)
        telemetry.addData("Arm Extension Power", armMotors.armExtension.power)
        telemetry.addData("Collector Power", armMotors.collector.power)
        telemetry.update()
    }

    override fun stop() {
        robot.stop()
    }

    private fun movement() {
        val x = gamepad1.left_stick_x.toDouble()
        val y = (-gamepad1.left_stick_y).toDouble()

        var powerLeft = y + x
        var powerRight = -y + x

        val max = maxOf(powerLeft, powerRight)

        if (max > 1) {
            powerLeft /= max
            powerRight /= max
        }

        precisionModeOn = gamepad1.a

        if (precisionModeOn) {
            powerLeft *= PRECISION_MODE_MULTIPLIER
            powerRight *= PRECISION_MODE_MULTIPLIER
        }

        with(wheelMotors) {
            rightFront.power = powerRight * MOTOR_SPEED_MULTIPLIER
            leftFront.power = powerLeft * MOTOR_SPEED_MULTIPLIER
            rightBack.power = powerRight * MOTOR_SPEED_MULTIPLIER
            leftBack.power = powerLeft * MOTOR_SPEED_MULTIPLIER
        }
    }

    private fun strafe() {
        with(wheelMotors) {
            if (gamepad1.right_stick_x > 0) { // Strafe Right
                rightFront.power = rightFront.power + MOTOR_SPEED_STRAFE
                leftFront.power = leftFront.power + MOTOR_SPEED_STRAFE
                rightBack.power = rightBack.power - MOTOR_SPEED_STRAFE
                leftBack.power = leftBack.power - MOTOR_SPEED_STRAFE
            } else if (gamepad1.right_stick_x < 0) { // Strafe Left
                rightFront.power = rightFront.power - MOTOR_SPEED_STRAFE
                leftFront.power = leftFront.power - MOTOR_SPEED_STRAFE
                rightBack.power = rightBack.power + MOTOR_SPEED_STRAFE
                leftBack.power = leftBack.power + MOTOR_SPEED_STRAFE
            }
        }
    }

    private fun latching() {
        val latchingPower = when {
            gamepad1.dpad_up -> LATCH_SPEED
            gamepad1.dpad_down -> -LATCH_SPEED
            else -> 0.0
        }

        armMotors.latchMotor.power = latchingPower
    }

    private fun armMovement() {
        val armAnglePower = -gamepad2.left_stick_y * ARM_ANGLE_SPEED_MULTIPLIER
        armMotors.armAngle.power = armAnglePower

        val armExtensionPower = gamepad2.right_stick_y * ARM_EXTENSION_SPEED_MULTIPLIER
        armMotors.armExtension.power = armExtensionPower
    }

    private fun collector() {
        val collectorPower = when {
            gamepad2.right_bumper -> COLLECTOR_SPEED
            gamepad2.left_bumper -> -COLLECTOR_SPEED
            else -> 0.0
        }

        armMotors.collector.power = collectorPower
    }

    private companion object {
        private const val PRECISION_MODE_MULTIPLIER = 0.45
        private const val MOTOR_SPEED_MULTIPLIER = 0.8
        private const val MOTOR_SPEED_STRAFE = 0.6
        private const val ARM_EXTENSION_SPEED_MULTIPLIER = 0.75
        private const val ARM_ANGLE_SPEED_MULTIPLIER = 0.55
        private const val COLLECTOR_SPEED = 0.5
        private const val LATCH_SPEED = 1.0
    }
}
