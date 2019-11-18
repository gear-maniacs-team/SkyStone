package org.firstinspires.ftc.teamcode.teleop

import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DcMotor
import org.firstinspires.ftc.teamcode.TeamRobot
import org.firstinspires.ftc.teamcode.utils.fastLazy

@TeleOp(name = "SimpleTeleOp", group = "Good")
class SimpleTeleOp : OpMode()  {

    private val robot = TeamRobot()
    private val wheelMotors by fastLazy { robot.wheelsMotors }

    override fun init() {
        robot.init(hardwareMap)

        wheelMotors.setModeAll(DcMotor.RunMode.RUN_USING_ENCODER)
    }

    override fun loop() {
        movement()
        strafe()
        telemetry.update()
    }

    override fun stop() {
        robot.stop()
    }

    private fun movement() {
        val x = gamepad1.left_stick_x.toDouble()
        val y = (-gamepad1.left_stick_y).toDouble()

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

        val precisionModeOn = gamepad1.a

        if (precisionModeOn) {
            powerRightFront *= PRECISION_MODE_MULTIPLIER
            powerLeftFront *= PRECISION_MODE_MULTIPLIER
            powerRightBack *= PRECISION_MODE_MULTIPLIER
            powerLeftBack *= PRECISION_MODE_MULTIPLIER
        }

        wheelMotors.rightFront.power = powerRightFront * MOTOR_SPEED_MULTIPLIER
        wheelMotors.leftFront.power = powerLeftFront * MOTOR_SPEED_MULTIPLIER
        wheelMotors.rightBack.power = powerRightBack * MOTOR_SPEED_MULTIPLIER
        wheelMotors.leftBack.power = powerLeftBack * MOTOR_SPEED_MULTIPLIER

        telemetry.addData("Precision Mode On", "%b\n", precisionModeOn)
    }

    private fun strafe() {
        // Strafe Right
        while (gamepad1.right_stick_x > 0) {
            wheelMotors.rightFront.power = MOTOR_SPEED_STRAFE
            wheelMotors.leftFront.power = MOTOR_SPEED_STRAFE
            wheelMotors.rightBack.power = -MOTOR_SPEED_STRAFE
            wheelMotors.leftBack.power = -MOTOR_SPEED_STRAFE
        }

        // Strafe Left
        while (gamepad1.right_stick_x < 0) {
            wheelMotors.rightFront.power = -MOTOR_SPEED_STRAFE
            wheelMotors.leftFront.power = -MOTOR_SPEED_STRAFE
            wheelMotors.rightBack.power = MOTOR_SPEED_STRAFE
            wheelMotors.leftBack.power = MOTOR_SPEED_STRAFE
        }
    }

    private companion object {
        private const val PRECISION_MODE_MULTIPLIER = 0.45
        private const val MOTOR_SPEED_MULTIPLIER = 0.7
        private const val MOTOR_SPEED_STRAFE = 0.6
    }
}