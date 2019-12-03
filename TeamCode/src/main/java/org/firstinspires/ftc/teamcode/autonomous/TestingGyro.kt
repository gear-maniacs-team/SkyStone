package org.firstinspires.ftc.teamcode.autonomous

import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.TeamRobot
import org.firstinspires.ftc.teamcode.pid.RotationPidController
import org.firstinspires.ftc.teamcode.sensors.Gyro
import org.firstinspires.ftc.teamcode.utils.fastLazy
import org.firstinspires.ftc.teamcode.utils.getVelocityForRpmAndEncoderCycles
import org.firstinspires.ftc.teamcode.utils.multiplyVelocity
import org.opencv.core.Mat
import java.lang.Thread.sleep
import kotlin.concurrent.thread

@TeleOp(name = "TestingGyro")
class TestingGyro : OpMode() {

    private val robot = TeamRobot()
    private val gyro = Gyro()
    private val wheelMotors by fastLazy { robot.wheelsMotors }
    private val pid = RotationPidController(500.0, 0.01, 200.0, 11.1)
    @Volatile
    private var pidAngle = 0.0

    override fun init() {
        robot.init(hardwareMap)
        gyro.init(hardwareMap)

        with(pid) {
            setInputRange(-Math.PI / 2, Math.PI / 2)
            setOutputRange(-220.0, 220.0)
            target = Math.PI / 2
        }

        gyro.waitForCalibration()

        telemetry.addData("Gyro", "Calibrated")
        telemetry.update()
    }

    override fun start() {
        gyro.start()

        thread(start = true) {
            while (robot.isOpModeActive) {
                robot.update(telemetry)
                pidAngle = pid.compute(gyro.angle.toDouble())
                sleep(5)
            }
        }
    }

    override fun loop() {
        val modifier = when {
            gamepad1.dpad_up -> 1.0
            gamepad1.dpad_down -> -1.0
            else -> 0.0
        }

        when {
            gamepad1.x -> {
                pid.Kp += modifier
                if (modifier != 0.0) sleep(200)
            }
            gamepad1.a -> {
                pid.Ki += modifier
                if (modifier != 0.0) sleep(200)
            }
            gamepad1.b -> {
                pid.Kd += modifier
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

        with(wheelMotors) {
            rightFront.velocity = getVelocityForRpmAndEncoderCycles(pidAngle, 383.6)
            leftFront.velocity = getVelocityForRpmAndEncoderCycles(pidAngle, 383.6)
            rightBack.velocity = getVelocityForRpmAndEncoderCycles(pidAngle, 753.2)
            leftBack.velocity = getVelocityForRpmAndEncoderCycles(pidAngle, 753.2)
        }

        telemetry.addData("PID", pid.toString())
        telemetry.addData("PID Result", pidAngle)
        telemetry.addData("Current Angle", gyro.angle)
        telemetry.addData("Target Angle", pid.target)
        telemetry.update()
    }

    override fun stop() {
        robot.stop()
    }
}
