package org.firstinspires.ftc.teamcode.sensors

import com.qualcomm.hardware.bosch.BNO055IMU
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference
import org.firstinspires.ftc.teamcode.RobotPos
import org.firstinspires.ftc.teamcode.utils.IHardware
import org.firstinspires.ftc.teamcode.utils.IUpdatable
import java.util.concurrent.atomic.AtomicBoolean

class Gyro : IHardware, IUpdatable {

    private companion object {
        private const val PI_F = Math.PI.toFloat()
    }

    private val axesRef = AxesReference.EXTRINSIC
    private val angleOrder = AxesOrder.XYZ
    private val angleUnit = AngleUnit.RADIANS

    private lateinit var firstImu: BNO055IMU
    private lateinit var secondImu: BNO055IMU

    private var resetNextTime = AtomicBoolean(true)
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
        val newAngle = (firstAngle + secondAngle) / 2

        if (resetNextTime.get()) {
            resetNextTime.set(false)
            lastAngle = newAngle
            angle = 0f

            // TODO: Test this
            //val currentAngle = RobotPos.currentAngle
            RobotPos.currentAngle = 0.0
            //RobotPos.targetAngle -= currentAngle
            return
        }

        var deltaAngle = newAngle - lastAngle

        if (deltaAngle < -PI_F) // < -180
            deltaAngle += PI_F * 2
        else if (deltaAngle > PI_F) // > 180
            deltaAngle -= PI_F * 2

        angle += deltaAngle

        lastAngle = newAngle
    }

    fun resetAngle() {
        resetNextTime.set(true)
    }

    override fun start() {
        waitForCalibration()
    }

    override fun update() {
        updateAngleValue()
        RobotPos.currentAngle = angle.toDouble()
    }
}