package org.firstinspires.ftc.teamcode.sensors

import com.qualcomm.hardware.bosch.BNO055IMU
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference
import org.firstinspires.ftc.teamcode.TeamRobot
import org.firstinspires.ftc.teamcode.utils.Ranges

class Gyro {

    private companion object {
        private const val PI_F = Math.PI.toFloat()
    }

    private val reference = AxesReference.EXTRINSIC
    private val angleOrder = AxesOrder.XYZ
    private val angleUnit = AngleUnit.RADIANS

    private lateinit var firstImu: BNO055IMU
    private lateinit var secondImu: BNO055IMU

    private var lastAngle: Float = 0f
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
        while (!firstImu.isGyroCalibrated || !secondImu.isGyroCalibrated)
            Thread.sleep(10)
    }

    /*
     * We have to process the angle because the imu works in euler angles so the Z axis is
     * returned as 0 to 3.14 or 0 to -3.14 rolling back to -3.12 or +3.12 when rotation passes
     * 3.14 radians. We detect this transition and track the total cumulative angle of rotation.
     *
     * @return the average angle of both IMU sensors in Radians
     */
    private fun getAngleValue(): Float {
        var currentAngle = angle
        lastAngle = currentAngle

        // The third angle is the Z angle, which is needed for heading
        val firstAngle = firstImu.getAngularOrientation(reference, angleOrder, angleUnit).thirdAngle
        val secondAngle = secondImu.getAngularOrientation(reference, angleOrder, angleUnit).thirdAngle
        val newAngle = (firstAngle + secondAngle) / 2

        if (!Ranges.isRangeValid(newAngle, -PI_F, PI_F))
            return currentAngle

        var deltaAngle = newAngle - currentAngle

        if (deltaAngle < -PI_F)
            deltaAngle += PI_F * 2
        else if (deltaAngle > PI_F)
            deltaAngle -= PI_F * 2

        currentAngle += deltaAngle

        return currentAngle
    }

    fun start() {
        waitForCalibration()

        val robot = TeamRobot.getRobot()
        Thread {
            while (robot.isOpModeActive) {
                angle = getAngleValue()
                Thread.sleep(50)
            }
        }.start()
    }
}