package net.gearmaniacs.teamcode.hardware.sensors

import com.qualcomm.hardware.bosch.BNO055IMU
import com.qualcomm.robotcore.hardware.HardwareMap
import net.gearmaniacs.teamcode.RobotPos
import net.gearmaniacs.teamcode.utils.IHardware
import net.gearmaniacs.teamcode.utils.IUpdatable
import net.gearmaniacs.teamcode.utils.extensions.getDevice
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference

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

    private lateinit var imu: BNO055IMU
    private var lastAngle = 0f
    var updateGlobalAngle = true
    @Volatile
    var angle = 0f
        private set

    override fun init(hardwareMap: HardwareMap) {
        imu = hardwareMap.getDevice("imu")

        val parameters = BNO055IMU.Parameters().apply {
            mode = BNO055IMU.SensorMode.IMU
            angleUnit = BNO055IMU.AngleUnit.RADIANS
            accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC
            loggingEnabled = false
        }

        imu.initialize(parameters)
    }

    fun waitForCalibration() {
        while (!imu.isGyroCalibrated)
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
        val firstAngle = imu.getAngularOrientation(axesRef, angleOrder, angleUnit).thirdAngle

        val deltaAngle = computeAngle(lastAngle, firstAngle)

        angle += deltaAngle
        lastAngle = firstAngle
    }

    override fun start() {
        waitForCalibration()
    }

    override fun update() {
        updateAngleValue()
        if (updateGlobalAngle)
            RobotPos.currentAngle = angle.toDouble()
    }
}