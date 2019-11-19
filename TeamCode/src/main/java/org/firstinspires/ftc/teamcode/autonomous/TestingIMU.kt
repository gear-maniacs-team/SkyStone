package org.firstinspires.ftc.teamcode.autonomous

import com.qualcomm.hardware.bosch.BNO055IMU
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.TeamRobot
import org.firstinspires.ftc.teamcode.utils.fastLazy
import java.lang.Thread.sleep

@TeleOp(name = "TestingIMU")
class TestingIMU : OpMode() {
    private val robot = TeamRobot()
    private val wheelMotors by fastLazy { robot.wheelsMotors }
    private lateinit var imu: BNO055IMU

    override fun init() {
        val parameters = BNO055IMU.Parameters().apply {
            mode = BNO055IMU.SensorMode.IMU
            angleUnit = BNO055IMU.AngleUnit.RADIANS
            accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC
            loggingEnabled = false
        }

        imu = hardwareMap.get(BNO055IMU::class.java, "imu")

        imu.initialize(parameters)
        telemetry.addData("Mode", "calibrating...");
        telemetry.update();

        // make sure the imu gyro is calibrated before continuing.
        while (!imu.isGyroCalibrated) {
            sleep(10);
        }
        telemetry.addData("Mode", "waiting for start");
        telemetry.addData("imu calibration status", imu.calibrationStatus.toString());
        telemetry.update();
    }

    override fun loop() {
        telemetry.addData("Angle", imu.angularOrientation)
        telemetry.update()
    }
}