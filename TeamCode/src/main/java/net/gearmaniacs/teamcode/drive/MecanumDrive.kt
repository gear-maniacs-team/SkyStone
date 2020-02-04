package net.gearmaniacs.teamcode.drive

import com.acmerobotics.roadrunner.control.PIDCoefficients
import com.qualcomm.hardware.bosch.BNO055IMU
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotor.RunMode
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.PIDFCoefficients
import net.gearmaniacs.teamcode.drive.Drive.MOTOR_VELOCITY_F
import org.firstinspires.ftc.robotcore.external.Telemetry

class MecanumDrive(hardwareMap: HardwareMap, telemetry: Telemetry) : MecanumDriveBase(telemetry) {

    private val leftFront: DcMotorEx
    private val leftBack: DcMotorEx
    private val rightBack: DcMotorEx
    private val rightFront: DcMotorEx
    private val motors: List<DcMotorEx>
    private val imu: BNO055IMU = hardwareMap.get(BNO055IMU::class.java, "imu")

    override fun getPIDCoefficients(runMode: RunMode?): PIDCoefficients {
        val coefficients: PIDFCoefficients = leftFront.getPIDFCoefficients(runMode)
        return PIDCoefficients(coefficients.p, coefficients.i, coefficients.d)
    }

    override fun setPIDCoefficients(runMode: RunMode?, coefficients: PIDCoefficients) {
        for (motor in motors) {
            motor.setPIDFCoefficients(
                runMode, PIDFCoefficients(
                    coefficients.kP, coefficients.kI, coefficients.kD, MOTOR_VELOCITY_F
                )
            )
        }
    }

    override val wheelVelocities: List<Double>
        get() {
            val wheelVelocities = ArrayList<Double>(motors.size)
            for (motor in motors) {
                wheelVelocities.add(Drive.ticksToCm(motor.velocity))
            }
            return wheelVelocities
        }

    override fun setMotorPowers(frontLeft: Double, rearLeft: Double, rearRight: Double, frontRight: Double) {
        leftFront.power = frontLeft
        leftBack.power = rearLeft
        rightBack.power = rearRight
        rightFront.power = frontRight
    }

    override val rawExternalHeading: Double
        get() = imu.angularOrientation.firstAngle.toDouble()

    override fun getWheelPositions(): List<Double> {
        val positions = ArrayList<Double>(motors.size)
        for (motor in motors) {
            positions.add(Drive.ticksToCm(motor.currentPosition))
        }
        return positions
    }

    init {
        val parameters = BNO055IMU.Parameters()
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS
        imu.initialize(parameters)
        // TODO: if your hub is mounted vertically, remap the IMU axes so that the z-axis points
// upward (normal to the floor) using a command like the following:
// BNO055IMUUtil.remapAxes(imu, AxesOrder.XYZ, AxesSigns.NPN);
        leftFront = hardwareMap.get(DcMotorEx::class.java, "leftFront")
        leftBack = hardwareMap.get(DcMotorEx::class.java, "leftRear")
        rightBack = hardwareMap.get(DcMotorEx::class.java, "rightRear")
        rightFront = hardwareMap.get(DcMotorEx::class.java, "rightFront")
        motors = listOf(leftFront, leftBack, rightBack, rightFront)
        for (motor in motors) {
            motor.mode = RunMode.RUN_USING_ENCODER
            motor.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        }
//        if (Drive.RUN_USING_ENCODER && Drive.MOTOR_VELO_PID != null) {
//            setPIDCoefficients(RunMode.RUN_USING_ENCODER, Drive.MOTOR_VELO_PID)
//        }
    }
}