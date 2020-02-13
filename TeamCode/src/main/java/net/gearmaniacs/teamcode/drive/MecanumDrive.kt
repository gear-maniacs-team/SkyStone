package net.gearmaniacs.teamcode.drive

import com.acmerobotics.roadrunner.control.PIDCoefficients
import com.acmerobotics.roadrunner.localization.Localizer
import com.qualcomm.hardware.bosch.BNO055IMU
import com.qualcomm.robotcore.hardware.*
import com.qualcomm.robotcore.hardware.DcMotor.RunMode
import net.gearmaniacs.teamcode.RobotPos
import net.gearmaniacs.teamcode.drive.Drive.MOTOR_VELOCITY_F
import net.gearmaniacs.teamcode.hardware.sensors.Encoders
import net.gearmaniacs.teamcode.utils.extensions.getDevice

class MecanumDrive(hardwareMap: HardwareMap) : MecanumDriveBase() {

    private val leftFront: DcMotorEx
    private val leftRear: DcMotorEx
    private val rightRear: DcMotorEx
    private val rightFront: DcMotorEx
    private val motors: List<DcMotorEx>
    private val imu: BNO055IMU = hardwareMap.get(BNO055IMU::class.java, "imu")
    private var localizerInstance: Localizer = Encoders()

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
        leftRear.power = rearLeft
        rightRear.power = rearRight
        rightFront.power = frontRight
    }

    override val rawExternalHeading: Double
        get() = imu.angularOrientation.thirdAngle.toDouble()

    override fun getWheelPositions(): List<Double> {
        val positions = ArrayList<Double>(motors.size)
        for (motor in motors) {
            positions.add(Drive.ticksToCm(motor.currentPosition))
        }
        return positions
    }

    override var localizer: Localizer
        get() = localizerInstance
        set(value) {
            throw IllegalAccessError("Localizer broken")
        }

    init {
        (localizerInstance as Encoders).init(hardwareMap)
        val parameters = BNO055IMU.Parameters()
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS
        imu.initialize(parameters)
        // TODO: if your hub is mounted vertically, remap the IMU axes so that the z-axis points
// upward (normal to the floor) using a command like the following:
// BNO055IMUUtil.remapAxes(imu, AxesOrder.XYZ, AxesSigns.NPN);
        leftFront = hardwareMap.getDevice("left_front")
        leftRear = hardwareMap.getDevice("left_rear")
        rightRear = hardwareMap.getDevice("right_rear")
        rightFront = hardwareMap.getDevice("right_front")
        rightRear.direction = DcMotorSimple.Direction.REVERSE
        rightFront.direction = DcMotorSimple.Direction.REVERSE

        motors = listOf(leftFront, leftRear, rightRear, rightFront)
        for (motor in motors) {
            motor.mode = RunMode.RUN_USING_ENCODER
            motor.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        }
    }
}