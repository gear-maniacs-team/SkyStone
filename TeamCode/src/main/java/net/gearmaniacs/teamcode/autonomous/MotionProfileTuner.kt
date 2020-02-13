package net.gearmaniacs.teamcode.autonomous

import android.os.Environment
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DcMotor
import net.gearmaniacs.teamcode.TeamOpMode
import net.gearmaniacs.teamcode.hardware.motors.Wheels
import net.gearmaniacs.teamcode.hardware.sensors.Encoders
import net.gearmaniacs.teamcode.utils.RobotClock
import net.gearmaniacs.teamcode.utils.extensions.getCurrentPosition
import java.io.File

@TeleOp(name = "Speed Tester")
class MotionProfileTuner : TeamOpMode() {

    data class DataSample(val time: Double, val distance: Double, val speed: Double, val acceleration: Double)

    private val encoders = Encoders()
    private val wheels = Wheels()
    private val set = mutableSetOf<DataSample>()

    override fun init() {
        initRobot(listOf(encoders, wheels), listOf(encoders))
        wheels.setModeAll(DcMotor.RunMode.RUN_WITHOUT_ENCODER)
    }


    private var previousDistance = 0.0
    private var previousTime = 0.0
    private var previousSpeed = 0.0

    override fun start() {
        super.start()
        previousTime = RobotClock.seconds()
        Thread.sleep(100)
    }

    private var maxVel = 0.0
    private var maxAcc = 0.0
    private var motorSpeed = -1.0

    override fun loop() {
        if (gamepad1.a)
            motorSpeed = 0.0
        val distanceTravelled = Encoders.toCm(encoders.right.getCurrentPosition(robot.bulkData1).toDouble())
        val currentTime = RobotClock.seconds()
        val deltaTime = currentTime - previousTime
        if (deltaTime < 0.1) return
        val currentSpeed = (distanceTravelled - previousDistance) / deltaTime
        val currentAcceleration = (currentSpeed - previousSpeed) / deltaTime
        maxVel = maxOf(maxVel, currentSpeed)
        maxAcc = maxOf(maxAcc, currentAcceleration)
        telemetry.addData("Distance", distanceTravelled)
        telemetry.addData("Speed", currentSpeed)
        telemetry.addData("Acceleration", currentAcceleration)
        telemetry.addData("Max Vel", maxVel)
        telemetry.addData("Max Acc", maxAcc)
        wheels.setPowerAll(motorSpeed)
        set.add(DataSample(currentTime, distanceTravelled, currentSpeed, currentAcceleration))

        previousDistance = distanceTravelled
        previousSpeed = currentSpeed
        previousTime = RobotClock.seconds()
    }

    override fun stop() {
        super.stop()
        val stringBuilder = StringBuilder()

        set.forEach {
            stringBuilder.append(it.time)
                .append(',')
                .append(it.distance)
                .append(',')
                .append(it.speed)
                .append(',')
                .append(it.acceleration)
                .append('\n')
        }

        File(Environment.getExternalStorageDirectory(), "motion.csv").writeText(stringBuilder.toString())
    }
}
