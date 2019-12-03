package org.firstinspires.ftc.teamcode.autonomous

import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.TeamRobot
import org.firstinspires.ftc.teamcode.pid.GeneralPidController
import org.firstinspires.ftc.teamcode.sensors.Gyro
import org.firstinspires.ftc.teamcode.utils.UPSCounter
import org.firstinspires.ftc.teamcode.utils.fastLazy
import org.firstinspires.ftc.teamcode.utils.getVelocityForRpmAndEncoderCycles
import java.lang.Thread.sleep
import kotlin.concurrent.thread

@TeleOp(name = "TestingGyro")
class TestingGyro : OpMode() {

    private val robot = TeamRobot()
    private val gyro = Gyro()
    private val wheelMotors by fastLazy { robot.wheelsMotors }
    private val pid = GeneralPidController(500.0, 0.01, 200.0)
    @Volatile
    private var pidResult = 0.0

    override fun init() {
        robot.init(hardwareMap)
        gyro.init(hardwareMap)

        with(pid) {
            setInputRange(0.0, Math.PI)
            setOutputRange(-220.0, 220.0)
            target = Math.PI / 2
            debugEnabled = true
        }

        gyro.waitForCalibration()

        telemetry.addData("Gyro", "Calibrated")
        telemetry.update()
    }

    override fun start() {
        gyro.start()

        val upsCounter = UPSCounter()

        thread(start = true) {
            while (robot.isOpModeActive) {
                upsCounter.update(telemetry)
                pidResult = pid.compute(gyro.angle.toDouble())
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

        var valueModified = true
        when {
            gamepad1.x -> pid.Kp += modifier
            gamepad1.a -> pid.Ki += modifier
            gamepad1.b -> pid.Kd += modifier
            gamepad1.y -> {
                val deltaAngle = Math.PI * modifier

                if (deltaAngle != 0.0) {
                    val newTarget = (pid.target + deltaAngle) % Math.PI
                    pid.target = newTarget
                }
            }
            else -> valueModified = false
        }

        if (modifier != 0.0 && valueModified)
            sleep(200) // Make sure that the value doesn't get updated several times

        with(wheelMotors) {
            rightFront.velocity = getVelocityForRpmAndEncoderCycles(pidResult, 383.6)
            leftFront.velocity = getVelocityForRpmAndEncoderCycles(pidResult, 383.6)
            rightBack.velocity = getVelocityForRpmAndEncoderCycles(pidResult, 753.2)
            leftBack.velocity = getVelocityForRpmAndEncoderCycles(pidResult, 753.2)
        }

        telemetry.addData("PID", pid.toString())
        telemetry.addData("PID Result", pidResult)
        telemetry.addData("Current Angle", gyro.angle)
        telemetry.addData("Target Angle", pid.target)
        telemetry.update()
    }

    override fun stop() {
        robot.stop()
    }
}
