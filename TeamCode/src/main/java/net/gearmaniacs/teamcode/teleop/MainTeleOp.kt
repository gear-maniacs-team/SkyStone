package net.gearmaniacs.teamcode.teleop

import android.util.Log
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.Servo
import net.gearmaniacs.teamcode.RobotPos
import net.gearmaniacs.teamcode.TeamOpMode
import net.gearmaniacs.teamcode.hardware.motors.Intake
import net.gearmaniacs.teamcode.hardware.motors.Lift
import net.gearmaniacs.teamcode.hardware.motors.Wheels
import net.gearmaniacs.teamcode.hardware.servos.FoundationServos
import net.gearmaniacs.teamcode.hardware.servos.OuttakeServos
import net.gearmaniacs.teamcode.pid.PidController
import net.gearmaniacs.teamcode.utils.*
import kotlin.concurrent.thread
import kotlin.math.*

abstract class MainTeleOp : TeamOpMode() {

    private val performanceProfiler = PerformanceProfiler()
    protected val wheels = Wheels()
    protected val intake = Intake()
    protected val lift = Lift()
    protected val foundation = FoundationServos()
    protected val outtake = OuttakeServos()

    private lateinit var gripper: Servo
    private lateinit var spinner: Servo

    private var liftTargetPosition = 0
    private var precisionModeOn = false

    private var rightFrontPower = 0.0
    private var leftFrontPower = 0.0
    private var rightBackPower = 0.0
    private var leftBackPower = 0.0

    private val headingIndependentDrive = DelayedBoolean(400)
    private var resetAngle = 0.0
    private var resetStrafePid = true
    private var curvedMovement = false
    private val strafePid = PidController(0.45, 0.0001, 0.5).apply {
        setOutputRange(-0.35, 0.35)
    }

    override fun init() {
        check(robot.isOpModeActive) { "TeamRobot::init must be called in all child classes" }
        RobotPos.resetAll()
        wheels.setModeAll(DcMotor.RunMode.RUN_WITHOUT_ENCODER)

        gripper = hardwareMap.getDevice("gripper")
        spinner = hardwareMap.getDevice("spinner")

        wheels.setModeAll(DcMotor.RunMode.RUN_USING_ENCODER)
    }

    override fun start() {
        super.start()

        thread {
            while (robot.isOpModeActive) {
                intake()
//                lift()
                outtake()
                foundation()
                Thread.yield()
            }
        }
    }

    override fun loop() {
        performanceProfiler.update(telemetry)

        precisionModeOn = gamepad1.right_bumper
        if (gamepad1.b) headingIndependentDrive.invert()

        if (gamepad1.left_stick_button && gamepad1.right_stick_button) {
            resetAngle = RobotPos.currentAngle
            Thread.sleep(300L)
        }

        rightFrontPower = 0.0
        leftFrontPower = 0.0
        rightBackPower = 0.0
        leftBackPower = 0.0

        testedMovement()
//        curveMovement()
//        planeMovement()

        with(wheels) {
            leftFront.power = leftFrontPower
            leftBack.power = leftBackPower
            rightBack.power = rightBackPower
            rightFront.power = rightFrontPower
        }

        with(telemetry) {
            addData("Heading Independent", headingIndependentDrive.value)
            addLine("--")
            addData("X Pos", "%3f", RobotPos.currentX)
            addData("Y Pos", "%3f", RobotPos.currentY)
            addData("Degrees", "%3f", Math.toDegrees(MathUtils.angleWrap(RobotPos.currentAngle)))
            addLine("--")
            addData("Power rightFront", "%6f", rightFrontPower)
            addData("Power leftFront", "%6f", leftFrontPower)
            addData("Power rightBack", "%6f", rightBackPower)
            addData("Power leftBack", "%6f", leftBackPower)
            addData("Lift Target Position", liftTargetPosition)
        }
    }

    private fun curveMovement() {
        val x = gamepad1.right_stick_x.toDouble()
        val y = expo(-gamepad1.right_stick_y.toDouble(), 1.0)

        if (abs(x) == 0.0 && abs(y) == 0.0)
            return

        var speedLeft = y + x
        var speedRight = -y + x

        val max = maxOf(speedLeft, speedRight)

        if (max > 1) {
            speedLeft /= max
            speedRight /= max
        }

        curvedMovement = true
        resetStrafePid = true

        val percentage = if (precisionModeOn) WHEELS_SPEED_PRECISION else WHEELS_SPEED_NORMAL

        speedRight *= percentage
        speedLeft *= percentage

        leftFrontPower -= speedLeft
        leftBackPower -= speedLeft
        rightBackPower += speedRight
        rightFrontPower += speedRight
    }

    private fun planeMovement() {
        val x = expo(gamepad1.left_stick_x.toDouble(), 1.0)
        val y = -expo(gamepad1.left_stick_y.toDouble(), 1.0)

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

        val magnitude = hypot(x, y) * if (precisionModeOn) WHEELS_SPEED_PRECISION else 1.0

        val independentAngleCorrection = if (headingIndependentDrive.value) RobotPos.currentAngle - resetAngle else 0.0
        val angle = atan2(y, x) - independentAngleCorrection
        val speedX = magnitude * sin(angle + Math.PI / 4)
        val speedY = magnitude * sin(angle - Math.PI / 4)

        telemetry.addData("Correction", correction)

        leftFrontPower += -speedY + correction
        leftBackPower += -speedX + correction
        rightBackPower += speedY + correction
        rightFrontPower += speedX + correction
    }

    private fun testedMovement() {
        val x = expo(gamepad1.left_stick_x.toDouble(), 1.0)
        val y = -expo(gamepad1.left_stick_y.toDouble(), 1.0)
        val theta = expo(gamepad1.right_stick_x.toDouble(), 1.0)

        if (abs(x) == 0.0 && abs(y) == 0.0) {
            resetStrafePid = true
            if (abs(theta) == 0.0)
                return
        }

        val sinOrientation = sin(RobotPos.currentAngle)
        val cosOrientation = cos(RobotPos.currentAngle)

        val fieldOrientedX = x * cosOrientation - y * sinOrientation
        val fieldOrientedY = x * sinOrientation + y * cosOrientation

        val curvedMovement = abs(theta) != 0.0

        if (resetStrafePid && !curvedMovement) {
            strafePid.reset()
            strafePid.setPoint = RobotPos.currentAngle
            resetStrafePid = false
        }

        val angle = if (curvedMovement) DRIVE_BASE_CONSTANT * theta else strafePid.compute(RobotPos.currentAngle)

        telemetry.addData("Angle Correction", angle)

        leftFrontPower += -fieldOrientedX - fieldOrientedY - angle
        leftBackPower += fieldOrientedX - fieldOrientedY - angle
        rightBackPower += -fieldOrientedX - fieldOrientedY + angle
        rightFrontPower += fieldOrientedX - fieldOrientedY + angle
    }

    private fun intake() {
        val intakePower = when {
            gamepad2.right_bumper -> INTAKE_POWER
            gamepad2.left_bumper -> -INTAKE_POWER * 0.8
            else -> 0.0
        }

        intake.setPowerAll(intakePower)
    }

    private fun lift() {
        val posChange = when {
            gamepad2.dpad_down -> -20
            gamepad2.dpad_up -> 20
            else -> 0
        }

        liftTargetPosition += posChange

        // Let's be honest, we can't trust the drivers
        if (liftTargetPosition > MAX_LIFT_HEIGHT_TICKS)
            liftTargetPosition = MAX_LIFT_HEIGHT_TICKS
        if (liftTargetPosition < 0)
            liftTargetPosition = 0

        if (posChange != 0)
            Thread.sleep(200)

        val power = if (liftTargetPosition == 0 && lift.left.currentPosition < 300 && lift.right.currentPosition < 300)
            0.0 else LIFT_POWER

        lift.setTargetPositionAll(liftTargetPosition)
        lift.setPowerAll(power)
    }

    private fun outtake() {
        if (gamepad2.y)
            outtake.extend()
        else if (gamepad2.x)
            outtake.retract()

        gripper.position = if (gamepad2.right_trigger > 0) 1.0 else 0.55

        if (gamepad2.a)
            spinner.position = 0.15
        else if (gamepad2.b)
            spinner.position = 0.97

        if (gamepad2.right_stick_button) {
            outtake.extend()
            Thread.sleep(600)
            spinner.position = 0.15
        }

        if (gamepad2.left_stick_button) {
            spinner.position = 0.97
            Thread.sleep(400)
            outtake.retract()
        }
    }

    private fun foundation() {
        if (gamepad2.left_trigger > 0)
            foundation.attach()
        else
            foundation.detach()
    }

    private companion object {
        private const val WHEELS_SPEED_PRECISION = 0.3
        private const val WHEELS_SPEED_NORMAL = 0.7
        private const val WHEELS_SPEED_TURBO = 0.8
        private const val INTAKE_POWER = 1.0
        private const val LIFT_POWER = 0.5
        private const val DRIVE_BASE_CONSTANT = 0.205 // drive base length/2 + drive base width/2
        private const val MAX_LIFT_HEIGHT_TICKS = 1200

        private fun expo(input: Double, expoFactor: Double): Double =
            expoFactor * input * input * input + (1 - expoFactor) * input
    }
}
