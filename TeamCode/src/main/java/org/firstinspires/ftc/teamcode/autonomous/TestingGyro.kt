package org.firstinspires.ftc.teamcode.autonomous

import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.TeamRobot
import org.firstinspires.ftc.teamcode.pid.RotationPidController
import org.firstinspires.ftc.teamcode.sensors.Gyro
import org.firstinspires.ftc.teamcode.utils.fastLazy
import org.firstinspires.ftc.teamcode.utils.getVelocityForRpmAndEncoderCycles
import org.firstinspires.ftc.teamcode.utils.multiplyVelocity
import java.lang.Thread.sleep

@TeleOp(name = "TestingGyro")
class TestingGyro : OpMode() {

    private val robot = TeamRobot()
    private val gyro = Gyro()
    private val wheelMotors by fastLazy { robot.wheelsMotors }
    private val pid = RotationPidController(12.0, 24.0, 1.5)

    //private val maxFrontVelocity = getVelocityForRpmAndEncoderCycles(100, 383.6)
    //private val maxBackVelocity = getVelocityForRpmAndEncoderCycles(100, 753.2)

    override fun init() {
        robot.init(hardwareMap)
        gyro.init(hardwareMap)

        with(pid) {
            setInputRange(0.0, Math.PI * 2)
            setOutputRange(-100.0, 100.0)
            target = Math.PI / 2
        }

        gyro.waitForCalibration()
    }

    override fun start() {
        gyro.start()
    }

    override fun loop() {
        val currentAngle = gyro.angle.toDouble()

        val modifier = when {
            gamepad1.dpad_up -> 10.0
            gamepad1.dpad_down -> 1 / 10.0
            else -> 1.0
        }

        when {
            gamepad1.x -> {
                pid.Kp *= modifier
                if (modifier != 0.0) sleep(200)
            }
            gamepad1.a -> {
                pid.Ki *= modifier
                if (modifier != 0.0) sleep(200)
            }
            gamepad1.b -> {
                pid.Kd *= modifier
                if (modifier != 0.0) sleep(200)
            }
            gamepad1.y -> {
                val target = when {
                    gamepad1.dpad_up -> Math.PI / 2
                    gamepad1.dpad_down -> -Math.PI / 2
                    else -> 0.0
                }
                pid.target += target
                pid.target %= Math.PI
                if (target != 0.0) sleep(200)
            }
        }

        val result = pid.compute(currentAngle)

        with(wheelMotors) {
            rightFront.velocity = getVelocityForRpmAndEncoderCycles(result, 383.6) //multiplyVelocity(maxFrontVelocity, result)
            leftFront.velocity = getVelocityForRpmAndEncoderCycles(result, 383.6) //multiplyVelocity(maxFrontVelocity, result)
            rightBack.velocity = getVelocityForRpmAndEncoderCycles(result, 753.2) //multiplyVelocity(maxBackVelocity, result)
            leftBack.velocity = getVelocityForRpmAndEncoderCycles(result, 753.2) //multiplyVelocity(maxBackVelocity, result)
        }

        telemetry.addData("PID", pid.toString())
        telemetry.addData("PID Result", result)
        telemetry.addData("Current Angle", currentAngle)
        telemetry.addData("Target Angle", pid.target)
        telemetry.update()
    }

    override fun stop() {
        robot.stop()
    }
}
