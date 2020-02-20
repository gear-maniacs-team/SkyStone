package net.gearmaniacs.teamcode.hardware.sensors

import android.util.Log
import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.localization.Localizer
import com.qualcomm.hardware.bosch.BNO055IMU
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.util.ReadWriteFile
import net.gearmaniacs.teamcode.RobotPos
import net.gearmaniacs.teamcode.utils.IHardware
import net.gearmaniacs.teamcode.utils.IUpdatable
import net.gearmaniacs.teamcode.utils.PerformanceProfiler
import net.gearmaniacs.teamcode.utils.extensions.epsilonEquals
import net.gearmaniacs.teamcode.utils.extensions.getDevice
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference
import org.firstinspires.ftc.robotcore.internal.system.AppUtil
import java.util.concurrent.Callable
import java.util.concurrent.Executors
import kotlin.math.cos
import kotlin.math.sin

/**
 * A combination of the [Gyro] and [Encoders] classes to allow Odometry using only two wheels
 */
class GyroEncoders : IHardware, IUpdatable, Localizer {

    private val axesRef = AxesReference.EXTRINSIC
    private val angleOrder = AxesOrder.XYZ
    private val angleUnit = AngleUnit.RADIANS

    private lateinit var imu: BNO055IMU
    lateinit var right: DcMotorEx
        private set
    lateinit var back: DcMotorEx
        private set

    private val performance = PerformanceProfiler()
    private val executor = Executors.newSingleThreadExecutor()

    private var previousRightPosition = 0.0
    private var previousBackPosition = 0.0
    private var lastAngle = 0.0

    var showUpdateTime = true

    private fun encodersUpdate(rightPos: Double, backPos: Double, deltaAngle: Double) {
        val currentAngle = RobotPos.currentAngle

        val deltaBack = Encoders.toCm(backPos - previousBackPosition)
        val deltaRight = Encoders.toCm(rightPos - previousRightPosition)

        var newX = deltaBack
        var newY = deltaRight
        if (!(deltaAngle epsilonEquals 0.0)) {
            val sinDeltaAngle = sin(deltaAngle / 2.0)
            newX = 2.0 * sinDeltaAngle * (deltaBack / deltaAngle + Encoders.DISTANCE_BETWEEN_BACK_ENCODER_AND_CENTER)
            newY = 2.0 * sinDeltaAngle * (deltaRight / deltaAngle + Encoders.DISTANCE_BETWEEN_ENCODER_WHEELS / 2)
        }

        val averageOrientation = -(currentAngle + deltaAngle / 2.0)

        val sinAverageOrientation = sin(averageOrientation)
        val cosAverageOrientation = cos(averageOrientation)
        RobotPos.currentX += newX * cosAverageOrientation - newY * sinAverageOrientation
        RobotPos.currentY += newX * sinAverageOrientation + newY * cosAverageOrientation

        previousBackPosition = backPos
        previousRightPosition = rightPos
    }

    override fun init(hardwareMap: HardwareMap) {
        // Init Gyro
        imu = hardwareMap.getDevice("imu")

        val parameters = BNO055IMU.Parameters().apply {
            mode = BNO055IMU.SensorMode.IMU
            angleUnit = BNO055IMU.AngleUnit.RADIANS
            accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC
            loggingEnabled = false
        }

        imu.initialize(parameters)

        // Init Encoders
        right = hardwareMap.getDevice("intake_right")
        back = hardwareMap.getDevice("intake_left")

        right.direction = DcMotorSimple.Direction.FORWARD
        back.direction = DcMotorSimple.Direction.FORWARD

        setModeAll(DcMotor.RunMode.STOP_AND_RESET_ENCODER)
        setModeAll(DcMotor.RunMode.RUN_WITHOUT_ENCODER)
    }

    fun waitForCalibration() {
        while (!imu.isGyroCalibrated)
            Thread.sleep(10)
    }

    private fun setModeAll(mode: DcMotor.RunMode) {
        right.mode = mode
        back.mode = mode
    }

    private fun updateAngleValue(): Double {
        // The third angle is the Z angle, which is needed for heading
        val newAngle = imu.getAngularOrientation(axesRef, angleOrder, angleUnit).thirdAngle.toDouble()

        val deltaAngle = Gyro.computeAngle(lastAngle, newAngle)

        RobotPos.currentAngle -= deltaAngle
        lastAngle = newAngle

        return -deltaAngle
    }

    override fun start() {
        waitForCalibration()
    }

    override fun update() {
        val future = executor.submit(Callable<Double> {
            updateAngleValue()
        })

        val rightPos = right.currentPosition.toDouble()
        val backPos = back.currentPosition.toDouble()
        val deltaAngle = future.get()

        encodersUpdate(rightPos, backPos, deltaAngle)

        if (showUpdateTime) {
            val ms = performance.update()
            Log.v("GyroEncoders", ms.toFloat().toString())
        }
    }

    // Please do not question this
    override var poseEstimate: Pose2d
        get() = Pose2d(RobotPos.currentY, -RobotPos.currentX, RobotPos.currentAngle)
        set(value) { RobotPos.currentX = value.y; RobotPos.currentY = -value.x; RobotPos.currentAngle = value.heading }
}
