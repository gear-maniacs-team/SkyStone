package org.firstinspires.ftc.teamcode.teleop

import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DistanceSensor
import com.qualcomm.robotcore.hardware.Servo
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit
import org.firstinspires.ftc.teamcode.RobotPos
import org.firstinspires.ftc.teamcode.TeamRobot
import org.firstinspires.ftc.teamcode.motors.Intake
import org.firstinspires.ftc.teamcode.motors.Wheels
import org.firstinspires.ftc.teamcode.pid.PidController
import org.firstinspires.ftc.teamcode.sensors.Gyro
import org.firstinspires.ftc.teamcode.utils.Ranges
import kotlin.concurrent.thread
import kotlin.math.*

@TeleOp(name = "G.E.A.R.S.")
class DemoTeleOp : OpMode() {

    private val robot = TeamRobot()
    private val wheels = Wheels()
    private val intake = Intake()
    private val gyro = Gyro()
    private lateinit var distanceSensor: DistanceSensor

    private val strafePid = PidController(256.0, 0.0001, 0.5).apply {
        tolerance = 0.00001
        setOutputRange(-100.0, 100.0)
    }

    private var releaseServo: Servo? = null
    private var cargoServo: Servo? = null

    private var precisionModeOn = false
    private var curvedMovement = false
    private var rightFrontVelocity = 0.0
    private var leftFrontVelocity = 0.0
    private var rightBackVelocity = 0.0
    private var leftBackVelocity = 0.0

    private var resetAngle = 0.0
    private var orientationIndependentDrive = false
    private var resetStrafePid = true

    private var distanceToStone = 0.0

    override fun init() {
        robot.init(hardwareMap, listOf(wheels, intake, gyro), listOf(gyro))

        wheels.setModeAll(DcMotor.RunMode.RUN_USING_ENCODER)
        wheels.setZeroPowerBehaviorAll(DcMotor.ZeroPowerBehavior.BRAKE)

        releaseServo = hardwareMap.servo["front"]
        cargoServo = hardwareMap.servo["cargo"]
        distanceSensor = hardwareMap.get(DistanceSensor::class.java, "cargo_distance")
    }

    override fun start() {
        robot.start()

        //releaseServo?.position = 0.0

        thread {
            while (robot.isOpModeActive) {
                distanceToStone = distanceSensor.getDistance(DistanceUnit.CM)
                intake()
                Thread.sleep(10L)
            }
        }
    }

    override fun loop() {
        telemetry.addData("Distance", distanceToStone)

        precisionModeOn = gamepad1.right_bumper
        if (gamepad1.b) {
            orientationIndependentDrive = !orientationIndependentDrive
            Thread.sleep(200L)
        }

        if (gamepad1.left_stick_button && gamepad1.right_stick_button) {
            resetAngle = RobotPos.currentAngle
            Thread.sleep(200L)
        }

        curvedMovement = false
        rightFrontVelocity = 0.0
        leftFrontVelocity = 0.0
        rightBackVelocity = 0.0
        leftBackVelocity = 0.0

        curveMovement()
        planeMovement()

        with(wheels) {
            rightFront.velocity = min(rightFrontVelocity, MAX_FRONT_VELOCITY)
            leftFront.velocity = min(leftFrontVelocity, MAX_FRONT_VELOCITY)
            rightBack.velocity = min(rightBackVelocity, MAX_BACK_VELOCITY)
            leftBack.velocity = min(leftBackVelocity, MAX_BACK_VELOCITY)
        }

        cargo()

        with(telemetry) {
            addData("Current Angle", RobotPos.currentAngle)
            addData("Target Angle", strafePid.setPoint)
            update()
        }
    }

    override fun stop() {
        robot.stop()
    }

    private fun curveMovement() {
        val x = -gamepad1.right_stick_x.toDouble()
        val y = gamepad1.right_stick_y.toDouble()

        if (abs(x) == 0.0 && abs(y) == 0.0)
            return

        var speedLeft = y + x
        var speedRight = -y + x

        val max = maxOf(speedLeft, speedRight)

        if (max > 1) {
            speedLeft /= max
            speedRight /= max
        }

        val percentage = if (precisionModeOn) PRECISION_MODE_MULTIPLIER else MOTOR_SPEED_MULTIPLIER

        speedRight *= percentage
        speedLeft *= percentage

        curvedMovement = true
        resetStrafePid = true

        rightFrontVelocity += getFrontVelocity(speedRight)
        leftFrontVelocity += getFrontVelocity(speedLeft)
        rightBackVelocity += getBackVelocity(speedRight)
        leftBackVelocity += getBackVelocity(speedLeft)
    }

    private fun planeMovement() {
        val x = -gamepad1.left_stick_x.toDouble()
        val y = gamepad1.left_stick_y.toDouble()

        if (abs(x) == 0.0 && abs(y) == 0.0) {
            resetStrafePid = true
            return
        }

        if (resetStrafePid && !curvedMovement) {
            strafePid.reset()
            strafePid.setPoint = RobotPos.currentAngle
            resetStrafePid = false
        }

        val correction = if (!curvedMovement) strafePid.compute(RobotPos.currentAngle) else 0.0
        val frontCorrection = Wheels.rpmToTps(correction, FRONT_ENCODER_COUNT)
        val backCorrection = Wheels.rpmToTps(correction, BACK_ENCODER_COUNT)

        val magnitude = hypot(x, y) * if (precisionModeOn) PRECISION_MODE_MULTIPLIER else MOTOR_SPEED_MULTIPLIER

        val independentAngleCorrection = if (orientationIndependentDrive) RobotPos.currentAngle - resetAngle else 0.0
        val angle = atan2(y, x) - Math.PI / 2 - independentAngleCorrection

        val speedX = magnitude * sin(angle + Math.PI / 4)
        val speedY = magnitude * sin(angle - Math.PI / 4)

        rightFrontVelocity += -getFrontVelocity(speedX) + frontCorrection
        leftFrontVelocity += -getFrontVelocity(speedY) + frontCorrection
        rightBackVelocity += getBackVelocity(speedY) + backCorrection
        leftBackVelocity += getBackVelocity(speedX) + backCorrection

        telemetry.addData("Strafe Correction", correction)
    }

    private fun autoIntake() {
        val power = if (distanceToStone < 16) INTAKE_POWER else 0.0
        intake.left.power = power
        intake.right.power = -power

        if (power != 0.0)
            Thread.sleep(1200L)
    }

    private fun intake() {
        // If the intake is not used by the driver, enter auto mode
        if (!(gamepad2.left_bumper || gamepad2.right_bumper || gamepad2.dpad_down || gamepad2.dpad_up)) {
            autoIntake()
            return
        }

        val intakePower = when {
            gamepad2.right_bumper -> INTAKE_POWER
            gamepad2.left_bumper -> -INTAKE_POWER * 0.8
            gamepad2.dpad_down -> INTAKE_POWER / 2
            gamepad2.dpad_up -> -INTAKE_POWER / 2
            else -> 0.0
        }

        intake.left.power = intakePower
        intake.right.power = -intakePower
    }

    private fun cargo() {
        cargoServo?.position = gamepad2.right_trigger.toDouble()
    }

    /*private fun extension() {
        if (gamepad2.y) {
            leftExtension.position = 0.0
            rightExtension.position = 0.0
        } else if (gamepad2.b) {
            leftExtension.position = 1.0
            rightExtension.position = 1.0
        }
    }*/

    private companion object {
        private const val MAX_RPM = 223.0
        private const val FRONT_ENCODER_COUNT = 383.6
        private const val BACK_ENCODER_COUNT = 753.2

        private const val PRECISION_MODE_MULTIPLIER = 0.45
        private const val MOTOR_SPEED_MULTIPLIER = 0.95
        private const val INTAKE_POWER = 0.7

        private fun getFrontVelocity(power: Double) =
            Wheels.rpmToTps(MAX_RPM * power, FRONT_ENCODER_COUNT)

        private fun getBackVelocity(power: Double) =
            Wheels.rpmToTps(MAX_RPM * power, BACK_ENCODER_COUNT)

        private val MAX_FRONT_VELOCITY = getFrontVelocity(1.0)
        private val MAX_BACK_VELOCITY = getBackVelocity(1.0)
    }
}
