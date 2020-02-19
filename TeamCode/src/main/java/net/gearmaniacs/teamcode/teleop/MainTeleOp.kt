package net.gearmaniacs.teamcode.teleop

import com.qualcomm.robotcore.hardware.DcMotor
import net.gearmaniacs.teamcode.RobotPos
import net.gearmaniacs.teamcode.TeamOpMode
import net.gearmaniacs.teamcode.drive.Drive
import net.gearmaniacs.teamcode.hardware.motors.Intake
import net.gearmaniacs.teamcode.hardware.motors.Lift
import net.gearmaniacs.teamcode.hardware.motors.Wheels
import net.gearmaniacs.teamcode.hardware.servos.FoundationServos
import net.gearmaniacs.teamcode.hardware.servos.OuttakeServos
import net.gearmaniacs.teamcode.pid.PidController
import net.gearmaniacs.teamcode.utils.DelayedBoolean
import net.gearmaniacs.teamcode.utils.MathUtils
import net.gearmaniacs.teamcode.utils.MathUtils.expo
import net.gearmaniacs.teamcode.utils.PerformanceProfiler
import net.gearmaniacs.teamcode.utils.extensions.coerceRange
import kotlin.concurrent.thread
import kotlin.math.abs
import kotlin.math.cos
import kotlin.math.sin

abstract class MainTeleOp : TeamOpMode() {

    private val performanceProfiler = PerformanceProfiler()
    protected val wheels = Wheels()
    protected val intake = Intake()
    protected val lift = Lift()
    protected val foundation = FoundationServos()
    protected val outtake = OuttakeServos()

    private var liftTargetPosition = 0
    private var precisionModeOn = false

    private var leftFrontPower = 0.0
    private var leftRearPower = 0.0
    private var rightRearPower = 0.0
    private var rightFrontPower = 0.0

    private var resetAngle = 0.0
    private var resetRotationPid = true
    private val rotationPid = PidController(0.4, 0.0, 0.0).apply {
        setOutputRange(-0.3, 0.3)
    }

    // Toggle Variables
    private val fieldOrientedToggle = Toggle()
    private val extensionToggle = Toggle()
    private val spinnerToggle = Toggle()
    private val gripperToggle = Toggle()
    private val foundationToggle = Toggle()

    override fun init() {
        check(robot.isOpModeActive) { "TeamRobot::init must be called in all child classes" }
        RobotPos.resetAll()

        wheels.setModeAll(DcMotor.RunMode.RUN_USING_ENCODER)
        gamepad1.setJoystickDeadzone(0f)
    }

    override fun start() {
        super.start()

        thread {
            while (robot.isOpModeActive) {
                secondaryLoop()
                Thread.yield()
            }
        }
    }

    override fun loop() {
        performanceProfiler.update(telemetry)

        precisionModeOn = gamepad1.right_bumper
        if (gamepad1.b) fieldOrientedToggle.invert()
        rotationPid.Kp = if (precisionModeOn) 0.0 else 0.4

        if (gamepad1.back) {
            resetAngle = RobotPos.currentAngle
            Thread.sleep(400L)
        }

        leftFrontPower = 0.0
        leftRearPower = 0.0
        rightRearPower = 0.0
        rightFrontPower = 0.0

        movement()

        with(wheels) {
            leftFront.velocity = Drive.cmToTicks(leftFrontPower.coerceRange(WHEELS_POWER_TOLERANCE) * Drive.MAX_VEL)
            leftRear.velocity = Drive.cmToTicks(leftRearPower.coerceRange(WHEELS_POWER_TOLERANCE) * Drive.MAX_VEL)
            rightRear.velocity = Drive.cmToTicks(rightRearPower.coerceRange(WHEELS_POWER_TOLERANCE) * Drive.MAX_VEL)
            rightFront.velocity = Drive.cmToTicks(rightFrontPower.coerceRange(WHEELS_POWER_TOLERANCE) * Drive.MAX_VEL)
        }

        with(telemetry) {
            addData("Field Oriented Enabled", fieldOrientedToggle.value)
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

        val currentAngle = if (fieldOrientedToggle.value) RobotPos.currentAngle - resetAngle else 0.0
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

    private fun secondaryLoop() {
        intake()
        lift()
        outtake()
        foundation()
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
            gamepad2.dpad_down -> -250
            gamepad2.dpad_up -> 250
            else -> 0
        }

        liftTargetPosition += posChange

        // Let's be honest, we can't trust the drivers
        if (liftTargetPosition > MAX_LIFT_HEIGHT_TICKS)
            liftTargetPosition = MAX_LIFT_HEIGHT_TICKS
        if (liftTargetPosition < 0)
            liftTargetPosition = 0

        if (gamepad2.back) {
            liftTargetPosition = 0
        }

        if (posChange != 0)
            Thread.sleep(200)

        lift.setTargetPositionAll(liftTargetPosition)
        lift.setPowerAll(LIFT_POWER)
    }

    private fun outtake() {
        if (gamepad2.start) {
            // Avoid executing command while initializing the controller
            return
        }

        if (gamepad2.y) extensionToggle.invert()
        if (gamepad2.b) spinnerToggle.invert()
        if (gamepad2.a) gripperToggle.invert()

        if (extensionToggle.value) outtake.extend() else outtake.retract()
        if (spinnerToggle.value) outtake.activateSpinner() else outtake.deactivateSpinner()
        if (gripperToggle.value) outtake.activateGripper() else outtake.releaseGripper()

        if (gamepad2.right_stick_button) {
            // Full outtake
            outtake.extend()
            Thread.sleep(600)
            outtake.activateSpinner()
            Thread.sleep(1000)
            outtake.semiExtend()

            extensionToggle.value = true
            spinnerToggle.value = true
        }

        if (gamepad2.left_stick_button) {
            // Full retract
            outtake.deactivateSpinner()
            Thread.sleep(450)
            outtake.retract()

            spinnerToggle.value = false
            extensionToggle.value = false
        }
    }

    private fun foundation() {
        if (gamepad2.x) foundationToggle.invert()
        if (foundationToggle.value) foundation.attach() else foundation.detach()
    }

    @Suppress("FunctionName")
    private companion object {
        private fun Toggle() = DelayedBoolean(400)

        private const val WHEELS_POWER_TOLERANCE = 0.05
        private const val WHEELS_SPEED_PRECISION = 0.4
        private const val WHEELS_SPEED_NORMAL = 0.9
        private const val INTAKE_POWER = 0.9
        private const val LIFT_POWER = 0.6
        private const val DRIVE_BASE_CONSTANT = 0.5 // measured in Maniacs
        private const val MAX_LIFT_HEIGHT_TICKS = 4500
    }
}
