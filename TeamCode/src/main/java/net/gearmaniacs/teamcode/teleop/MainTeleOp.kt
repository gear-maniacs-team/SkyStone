package net.gearmaniacs.teamcode.teleop

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
import net.gearmaniacs.teamcode.utils.MathUtils.expo
import net.gearmaniacs.teamcode.utils.extensions.coerceRange
import net.gearmaniacs.teamcode.utils.extensions.getDevice
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

    private var leftFrontPower = 0.0
    private var leftRearPower = 0.0
    private var rightRearPower = 0.0
    private var rightFrontPower = 0.0

    private val headingIndependentDrive = DelayedBoolean(400)
    private var resetAngle = 0.0
    private var resetRotationPid = true
    private val rotationPid = PidController(0.5, 0.0, 0.0).apply {
        setOutputRange(-0.3, 0.3)
    }

    override fun init() {
        check(robot.isOpModeActive) { "TeamRobot::init must be called in all child classes" }
        RobotPos.resetAll()
        wheels.setModeAll(DcMotor.RunMode.RUN_USING_ENCODER)

        gripper = hardwareMap.getDevice("gripper")
        spinner = hardwareMap.getDevice("spinner")

        wheels.setModeAll(DcMotor.RunMode.RUN_USING_ENCODER)
    }

    override fun start() {
        super.start()

        thread {
            while (robot.isOpModeActive) {
                intake()
                lift()
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
            Thread.sleep(400L)
        }

        if (precisionModeOn) {
            rotationPid.Kp = 0.0
        } else {
            rotationPid.Kp = 0.4
        }

        leftFrontPower = 0.0
        leftRearPower = 0.0
        rightRearPower = 0.0
        rightFrontPower = 0.0

        movement()

        with(wheels) {
            leftFront.power = leftFrontPower.coerceRange(WHEELS_POWER_TOLERANCE)
            leftRear.power = leftRearPower.coerceRange(WHEELS_POWER_TOLERANCE)
            rightRear.power = rightRearPower.coerceRange(WHEELS_POWER_TOLERANCE)
            rightFront.power = rightFrontPower.coerceRange(WHEELS_POWER_TOLERANCE)
        }

        with(telemetry) {
            addData("Heading Independent", headingIndependentDrive.value)
            addLine("---")
            addData("X Pos", "%3f", RobotPos.currentX)
            addData("Y Pos", "%3f", RobotPos.currentY)
            addData("Degrees", "%3f", Math.toDegrees(MathUtils.angleWrap(RobotPos.currentAngle)))
            addLine("---")
            addData("Power leftFront", "%5f", leftFrontPower)
            addData("Power leftRear", "%5f", leftRearPower)
            addData("Power rightRear", "%5f", rightRearPower)
            addData("Power rightFront", "%5f", rightFrontPower)
            addData("Lift Target Position", liftTargetPosition)
        }
    }

    private fun movement() {
        val x = expo(gamepad1.left_stick_x.toDouble(), 1.0)
        val y = -expo(gamepad1.left_stick_y.toDouble(), 1.0)
        val theta = expo(gamepad1.right_stick_x.toDouble(), 1.0)

        val powerMultiplier = if (precisionModeOn) WHEELS_SPEED_PRECISION else WHEELS_SPEED_NORMAL
        val curvedMovement = abs(theta) != 0.0

        val angleVariation = if (curvedMovement) {
            resetRotationPid = true
            DRIVE_BASE_CONSTANT * theta * powerMultiplier
        } else {
            if (resetRotationPid) {
                rotationPid.reset()
                rotationPid.setPoint = RobotPos.currentAngle
                resetRotationPid = false
            }
            rotationPid.compute(RobotPos.currentAngle)
        }

        leftFrontPower -= angleVariation
        leftRearPower -= angleVariation
        rightRearPower += angleVariation
        rightFrontPower += angleVariation

        telemetry.addData("Angle Variation", angleVariation)

        if (abs(x) == 0.0 && abs(y) == 0.0) return

        val currentAngle = if (headingIndependentDrive.value) RobotPos.currentAngle - resetAngle else 0.0
        val sinOrientation = sin(currentAngle)
        val cosOrientation = cos(currentAngle)

        val fieldOrientedX = x * cosOrientation - y * sinOrientation
        val fieldOrientedY = x * sinOrientation + y * cosOrientation

        val denominator = abs(fieldOrientedX) + abs(fieldOrientedY)
        var scaledX = fieldOrientedX * powerMultiplier
        var scaledY = fieldOrientedY * powerMultiplier

        if (denominator > 1.0) {
            scaledX /= denominator
            scaledY /= denominator
        }

        leftFrontPower += -scaledX - scaledY
        leftRearPower += scaledX - scaledY
        rightRearPower += -scaledX - scaledY
        rightFrontPower += scaledX - scaledY
    }

    private fun intake() {
        val intakePower = when {
            gamepad2.right_bumper -> INTAKE_POWER
            gamepad2.left_bumper -> -INTAKE_POWER * 0.9
            else -> 0.0
        }

        intake.setPowerAll(intakePower)
    }

    private fun lift() {
        val posChange = when {
            gamepad2.dpad_down -> -256
            gamepad2.dpad_up -> 256
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

        lift.setTargetPositionAll(liftTargetPosition)
        lift.setPowerAll(LIFT_POWER)
    }

    private fun outtake() {
        if (gamepad2.y)
            outtake.extend()
        else if (gamepad2.x)
            outtake.retract()

        gripper.position = if (gamepad2.right_trigger > 0) 0.8 else 0.5

        if (gamepad2.a)
            spinner.position = 0.15
        else if (gamepad2.b)
            spinner.position = 0.97

        if (gamepad2.right_stick_button) {
            outtake.extend()
            Thread.sleep(600)
            spinner.position = 0.15
            Thread.sleep(1000)
            outtake.semiExtend()
        }

        if (gamepad2.left_stick_button) {
            spinner.position = 0.87
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
        private const val WHEELS_POWER_TOLERANCE = 0.05
        private const val WHEELS_SPEED_PRECISION = 0.4
        private const val WHEELS_SPEED_NORMAL = 0.9
        private const val INTAKE_POWER = 0.85
        private const val LIFT_POWER = 0.5
        private const val DRIVE_BASE_CONSTANT = 0.5 // measured in Maniacs
        private const val MAX_LIFT_HEIGHT_TICKS = 3600
    }
}
