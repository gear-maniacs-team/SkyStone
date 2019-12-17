package org.firstinspires.ftc.teamcode.sensors

import com.qualcomm.hardware.bosch.BNO055IMU
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference
import org.firstinspires.ftc.teamcode.RobotPos
import org.firstinspires.ftc.teamcode.utils.IHardware
import org.firstinspires.ftc.teamcode.utils.IUpdatable

class Gyro : IHardware, IUpdatable {

    companion object {
        private const val PI_F = Math.PI.toFloat()

        private fun computeAngle(lastAngle: Float, newAngle: Float): Float {
            var deltaAngle = newAngle - lastAngle

            if (deltaAngle < -PI_F) // < -180
                deltaAngle += PI_F * 2
            else if (deltaAngle > PI_F) // > 180
                deltaAngle -= PI_F * 2

            return deltaAngle
        }
    }

    private val axesRef = AxesReference.EXTRINSIC
    private val angleOrder = AxesOrder.XYZ
    private val angleUnit = AngleUnit.RADIANS

    private lateinit var firstImu: BNO055IMU
    private lateinit var secondImu: BNO055IMU
    private var lastAngle = 0f
    private var angle = 0f

    override fun init(hardwareMap: HardwareMap) {
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
     * returned as 0 to 3.14 or 0 to -3.14 rolling back to -3.124 or +3.124 when rotation passes
     * 3.14 radians. We detect this transition and track the total cumulative angle of rotation.
     *
     * @return the average angle of both IMU sensors in Radians
     */
    private fun updateAngleValue() {
        // The third angle is the Z angle, which is needed for heading
        val firstAngle = firstImu.getAngularOrientation(axesRef, angleOrder, angleUnit).thirdAngle
        val secondAngle = secondImu.getAngularOrientation(axesRef, angleOrder, angleUnit).thirdAngle

        val deltaAngle1 = computeAngle(lastAngle, firstAngle)
        //val deltaAngle2 = computeAngle(lastAngle, secondAngle)

        //angle += (deltaAngle1 + deltaAngle2) / 2
        angle += deltaAngle1
        lastAngle = firstAngle
    }

    override fun start() {
        waitForCalibration()
    }

    override fun update() {
        updateAngleValue()
        RobotPos.currentAngle = angle.toDouble()
    }
}