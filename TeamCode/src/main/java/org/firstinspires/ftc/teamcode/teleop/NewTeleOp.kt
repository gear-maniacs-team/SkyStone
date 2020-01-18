package org.firstinspires.ftc.teamcode.teleop

import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.Servo
import com.qualcomm.robotcore.robot.Robot
import org.firstinspires.ftc.teamcode.RobotPos
import org.firstinspires.ftc.teamcode.TeamRobot
import org.firstinspires.ftc.teamcode.motors.Intake
import org.firstinspires.ftc.teamcode.motors.Wheels
import org.firstinspires.ftc.teamcode.pid.PidController
import org.firstinspires.ftc.teamcode.sensors.Encoder
import org.firstinspires.ftc.teamcode.utils.MathUtils
import org.firstinspires.ftc.teamcode.utils.PerformanceProfiler
import kotlin.concurrent.thread
import kotlin.math.abs
import kotlin.math.atan2
import kotlin.math.hypot
import kotlin.math.min
import kotlin.math.sin

@TeleOp(name = "Mark TeleOp")
class NewTeleOp : OpMode() {

    private val upsCounter = PerformanceProfiler()
    private val robot = TeamRobot()
    private val wheels = Wheels()
    private val intake = Intake()
    private val encoder = Encoder()

    private val strafePid = PidController(256.0, 0.0001, 0.5).apply {
        setOutputRange(-100.0, 100.0)
    }

    private var precisionModeOn = false
    private var curvedMovement = false
    private var rightFrontPower = 0.0
    private var leftFrontPower = 0.0
    private var rightBackPower = 0.0
    private var leftBackPower = 0.0

    private var resetAngle = 0.0
    private var orientationIndependentDrive = false
    private var resetStrafePid = true

    override fun init() {
        robot.init(hardwareMap, listOf(wheels), listOf())

        wheels.setZeroPowerBehaviorAll(DcMotor.ZeroPowerBehavior.BRAKE)
    }

    override fun start() {
        robot.start()
        RobotPos.resetAll()
    }

    override fun loop() {
        upsCounter.update(telemetry)

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
        rightFrontPower = 0.0
        leftFrontPower = 0.0
        rightBackPower = 0.0
        leftBackPower = 0.0

        curveMovement()
        planeMovement()

        with(wheels) {
            rightFront.power = min(rightFrontPower, MAX_MOTOR_POWER)
            leftFront.power = min(leftFrontPower, MAX_MOTOR_POWER)
            rightBack.power = min(rightBackPower, MAX_MOTOR_POWER)
            leftBack.power = min(leftBackPower, MAX_MOTOR_POWER)
        }

        with(telemetry) {
            addData("X", Encoder.ticksToCM(RobotPos.currentX))
            addData("Y", Encoder.ticksToCM(RobotPos.currentY))
            addData("Current Angle", MathUtils.angleWrap(RobotPos.currentAngle))
            addData("Target Angle", strafePid.setPoint)

            addData("rightFront power", wheels.rightFront.power)
            addData("leftFront power", wheels.leftFront.power)
            addData("rightBack power", wheels.rightBack.power)
            addData("leftBack power", wheels.leftBack.power)
            update()
        }
    }

    override fun stop() {
        robot.stop()
    }

    private fun curveMovement() {
        val x = gamepad1.right_stick_x.toDouble()
        val y = -gamepad1.right_stick_y.toDouble()

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

        rightFrontPower += speedRight
        leftFrontPower += speedLeft
        rightBackPower += speedRight
        leftBackPower += speedLeft
    }

    private fun planeMovement() {
        val x = gamepad1.left_stick_x.toDouble()
        val y = -gamepad1.left_stick_y.toDouble()

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
        val magnitude = hypot(x, y) * if (precisionModeOn) PRECISION_MODE_MULTIPLIER else MOTOR_SPEED_MULTIPLIER

        val independentAngleCorrection = if (orientationIndependentDrive) RobotPos.currentAngle - resetAngle else 0.0
        val angle = atan2(y, x) - Math.PI / 2 - independentAngleCorrection

        val speedX = magnitude * sin(angle + Math.PI / 4)
        val speedY = magnitude * sin(angle - Math.PI / 4)

        rightFrontPower += -speedX + correction
        leftFrontPower += -speedY + correction
        rightBackPower += speedY + correction
        leftBackPower += speedX + correction

        telemetry.addData("Strafe Correction", correction)
    }

    private fun intake() {
        // If the intake is not used by the driver, enter auto mode
        /*if (!(gamepad2.left_bumper || gamepad2.right_bumper || gamepad2.dpad_down || gamepad2.dpad_up)) {
            autoIntake()
            return
        }*/

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

    private companion object {
        private const val PRECISION_MODE_MULTIPLIER = 0.3
        private const val MOTOR_SPEED_MULTIPLIER = 0.6
        private const val MAX_MOTOR_POWER = 0.9
        private const val INTAKE_POWER = 0.7
    }
}
