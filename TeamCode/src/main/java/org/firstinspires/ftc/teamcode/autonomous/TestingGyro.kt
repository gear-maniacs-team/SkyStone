package org.firstinspires.ftc.teamcode.autonomous

import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.RobotPos
import org.firstinspires.ftc.teamcode.TeamRobot
import org.firstinspires.ftc.teamcode.motors.Wheels
import org.firstinspires.ftc.teamcode.pid.PidRotation
import org.firstinspires.ftc.teamcode.sensors.Gyro
import org.firstinspires.ftc.teamcode.utils.rpmToTps

@TeleOp(name = "TestingGyro")
class TestingGyro : OpMode() {

    private val robot = TeamRobot()
    private val wheels = Wheels()
    private val gyro = Gyro()
    private val pid = PidRotation().apply {
        setOutputRange(-223.0, 223.0)
        debugEnabled = true
    }

    override fun init() {
        robot.init(
            hardwareMap,
            listOf(wheels, gyro),
            listOf(gyro, pid)
        )

        gyro.waitForCalibration()

        telemetry.addData("Gyro", "Calibrated")
        telemetry.update()
    }

    override fun start() {
        robot.start()
    }

    override fun loop() {
        val pidOutput = pid.output
        val controller = pid.controller

        val modifier = when {
            gamepad1.dpad_up -> 1.0
            gamepad1.dpad_down -> -1.0
            else -> 0.0
        }

        var valueModified = true
        when {
            gamepad1.x -> controller.Kp += modifier
            gamepad1.a -> controller.Ki += modifier
            gamepad1.b -> controller.Kd += modifier
            gamepad1.y -> {
                if (modifier != 0.0) {
                    val deltaAngle = (Math.PI / 2) * modifier

                    gyro.resetAngle()
                    controller.reset()
                    RobotPos.targetAngle = deltaAngle
                    return
                }
            }
            else -> valueModified = false
        }

        if (modifier != 0.0 && valueModified)
            Thread.sleep(200) // Make sure that the value doesn't get updated several times

        with(wheels) {
            val frontVelocity = rpmToTps(pidOutput, 383.6)
            val backVelocity = rpmToTps(pidOutput, 753.2)

            rightFront.velocity = frontVelocity
            leftFront.velocity = frontVelocity
            rightBack.velocity = backVelocity
            leftBack.velocity = backVelocity
        }

        telemetry.addData("PID", pid.toString())
        telemetry.addData("PID Result", pidOutput)
        telemetry.addData("Current Angle", RobotPos.currentAngle)
        telemetry.addData("Target Angle", RobotPos.targetAngle)
        telemetry.update()
    }

    override fun stop() {
        robot.stop()
    }
}
