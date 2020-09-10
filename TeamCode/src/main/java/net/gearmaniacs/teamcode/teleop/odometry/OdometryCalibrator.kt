package net.gearmaniacs.teamcode.teleop.odometry

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DcMotor
import net.gearmaniacs.teamcode.RobotPos
import net.gearmaniacs.teamcode.TeamRobot
import net.gearmaniacs.teamcode.hardware.motors.Wheels
import net.gearmaniacs.teamcode.hardware.sensors.Encoders
import net.gearmaniacs.teamcode.hardware.sensors.Gyro


@TeleOp(name = "Calibrate Odometry", group = "Odometry")
class OdometryCalibrator : LinearOpMode() {

    private val encoders = Encoders()
    private val wheels = Wheels()
    private val gyro = Gyro()
    private val robot = TeamRobot()

    override fun runOpMode() {
        robot.useBulkRead = false
        robot.init(
            hardwareMap,
            listOf(wheels, gyro, encoders),
            listOf(gyro, encoders)
        )
        RobotPos.resetAll()

        wheels.setModeAll(DcMotor.RunMode.RUN_WITHOUT_ENCODER)
        wheels.setZeroPowerBehaviorAll(DcMotor.ZeroPowerBehavior.BRAKE)

        robot.start()

        telemetry.addLine("Initialization Successful")
        telemetry.update()

        wheels.leftFront.power = 0.5
        wheels.rightFront.power = 0.5
        wheels.leftRear.power = -0.5
        wheels.rightRear.power = -0.5

        Thread.sleep(SPIN_TIME)

        wheels.leftFront.power = 0.0
        wheels.rightFront.power = 0.0
        wheels.leftRear.power = 0.0
        wheels.rightRear.power = 0.0

        Thread.sleep(BREAK_TIME)

        val gyroAngle = gyro.angle
        val deltaLeft = encoders.left.currentPosition
        val deltaRight = encoders.right.currentPosition
        val computedDistance = (deltaLeft - deltaRight) / gyroAngle
        telemetry.addLine("Calibration complete")
        telemetry.addLine("Distance: $computedDistance")
        telemetry.update()
        Thread.sleep(10000)

        robot.stop()

    }

    private companion object {
        private const val SPIN_TIME = 5000L
        private const val BREAK_TIME = 2000L
    }


}