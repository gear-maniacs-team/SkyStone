package org.firstinspires.ftc.teamcode.sensors

import com.qualcomm.hardware.bosch.BNO055IMU
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference
import org.firstinspires.ftc.teamcode.TeamRobot

class Gyro {

    private val reference = AxesReference.EXTRINSIC
    private val angleOrder = AxesOrder.XYZ
    private val angleUnit = AngleUnit.RADIANS

    private lateinit var firstImu: BNO055IMU
    private lateinit var secondImu: BNO055IMU

    @Volatile
    var angle: Float = 0f
        private set

    fun init(hardwareMap: HardwareMap) {
        firstImu = hardwareMap.get(BNO055IMU::class.java, "imu_1")
        secondImu = hardwareMap.get(BNO055IMU::class.java, "imu_2")

        val parameters = BNO055IMU.Parameters().apply {
            mode = BNO055IMU.SensorMode.IMU
            angleUnit = BNO055IMU.AngleUnit.RADIANS
            accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC
            loggingEnabled = false
        }

        firstImu.initialize(parameters)
        secondImu.initialize(parameters)
    }

    fun waitForCalibration() {
        while (!firstImu.isGyroCalibrated || !secondImu.isGyroCalibrated) {
            Thread.sleep(20)
        }
    }

    private fun updateValue() {
        val firstAngle = firstImu.getAngularOrientation(reference, angleOrder, angleUnit).thirdAngle
        val secondAngle = secondImu.getAngularOrientation(reference, angleOrder, angleUnit).thirdAngle

        angle = (firstAngle + secondAngle) / 2
    }

    fun start() {
        waitForCalibration()

        val robot = TeamRobot.getRobot()
        Thread {
            while (robot.isOpModeActive) {
                updateValue()
                Thread.sleep(100)
            }
        }.start()
    }
}