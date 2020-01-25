package net.gearmaniacs.teamcode.teleop

import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.Servo
import net.gearmaniacs.teamcode.RobotPos
import net.gearmaniacs.teamcode.TeamRobot
import net.gearmaniacs.teamcode.motors.Intake
import net.gearmaniacs.teamcode.motors.Lift
import net.gearmaniacs.teamcode.motors.Wheels
import net.gearmaniacs.teamcode.pid.PidController
import net.gearmaniacs.teamcode.sensors.Encoder
import net.gearmaniacs.teamcode.servos.FoundationServos
import net.gearmaniacs.teamcode.servos.OuttakeServos
import net.gearmaniacs.teamcode.utils.MathUtils
import net.gearmaniacs.teamcode.utils.PerformanceProfiler
import net.gearmaniacs.teamcode.utils.getDevice
import kotlin.concurrent.thread
import kotlin.math.abs
import kotlin.math.atan2
import kotlin.math.hypot
import kotlin.math.sin

@TeleOp(name = "Mark TeleOp", group = "Good")
open class NewTeleOp : OpMode() {

    private val performanceProfiler = PerformanceProfiler()
    private val robot = TeamRobot()
    private val wheels = Wheels()
    private val intake = Intake()
    private val encoder = Encoder()
    private val lift = Lift()
    private val foundation = FoundationServos()
    private val outtake = OuttakeServos()

    private lateinit var gripper: Servo
    private lateinit var spinner: Servo

    private var liftTargetPosition = 0
    private var precisionModeOn = false
    private var rightFrontPower = 0.0
    private var leftFrontPower = 0.0
    private var rightBackPower = 0.0
    private var leftBackPower = 0.0

    private var resetAngle = 0.0
    private var orientationIndependentDrive = false
    private var resetStrafePid = true
    private var curvedMovement = false
    private val strafePid = PidController(256.0, 0.0001, 0.5).apply {
        setOutputRange(-100.0, 100.0)
    }

    override fun init() {
        robot.init(hardwareMap, listOf(wheels, encoder, intake, lift, foundation, outtake), listOf(encoder))
        wheels.setModeAll(DcMotor.RunMode.RUN_WITHOUT_ENCODER)

        gripper = hardwareMap.getDevice("gripper")
        spinner = hardwareMap.getDevice("spinner")
    }

    override fun start() {
        robot.start()
        RobotPos.resetAll()

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
        if (gamepad1.b) {
            orientationIndependentDrive = !orientationIndependentDrive
            Thread.sleep(200L)
        }

        if (gamepad1.left_stick_button && gamepad1.right_stick_button) {
            resetAngle = RobotPos.currentAngle
            Thread.sleep(200L)
        }

        rightFrontPower = 0.0
        leftFrontPower = 0.0
        rightBackPower = 0.0
        leftBackPower = 0.0

        curveMovement()
        planeMovement()

        with(wheels) {
            rightFront.power = rightFrontPower
            leftFront.power = leftFrontPower
            rightBack.power = rightBackPower
            leftBack.power = leftBackPower
        }

        with(telemetry) {
            addData("X", RobotPos.currentX)
            addData("Y", RobotPos.currentY)
            addData("Heading", Math.toDegrees(MathUtils.angleWrap(RobotPos.currentAngle)))
            addData("--", "--")
            addData("rightFront power", wheels.rightFront.power)
            addData("leftFront power", wheels.leftFront.power)
            addData("rightBack power", wheels.rightBack.power)
            addData("leftBack power", wheels.leftBack.power)
            addData("Lift Target Position", liftTargetPosition)

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

        curvedMovement = true
        resetStrafePid = true

        val percentage = if (precisionModeOn) PRECISION_MODE_MULTIPLIER else MOTOR_SPEED_MULTIPLIER

        speedRight *= percentage
        speedLeft *= percentage

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

        val correction = 0.0//if (!curvedMovement) strafePid.compute(RobotPos.currentAngle) else 0.0

        val magnitude = hypot(x, y) * if (precisionModeOn) PRECISION_MODE_MULTIPLIER else MOTOR_SPEED_MULTIPLIER

       // val independentAngleCorrection = if (orientationIndependentDrive) RobotPos.currentAngle - resetAngle else 0.0
        val angle = atan2(y, x) - Math.PI / 2 //- independentAngleCorrection
        val speedX = magnitude * sin(angle + Math.PI / 4)
        val speedY = magnitude * sin(angle - Math.PI / 4)

        rightFrontPower += -speedX + correction
        leftFrontPower += -speedY + correction
        rightBackPower += speedY + correction
        leftBackPower += speedX + correction
    }

    private fun intake() {
        val intakePower = when {
            gamepad2.right_bumper -> INTAKE_POWER
            gamepad2.left_bumper -> -INTAKE_POWER * 0.8
            else -> 0.0
        }

        intake.left.power = intakePower
        intake.right.power = -intakePower
    }

    private fun lift() {
        val posChange = when {
            gamepad2.dpad_down -> -120
            gamepad2.dpad_up -> 120
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

        with(lift) {
            left.targetPosition = liftTargetPosition
            right.targetPosition = -liftTargetPosition
            left.power = power
            right.power = power
        }
    }

    private fun outtake() {
        if (gamepad2.y)
            outtake.extend()
        else if (gamepad2.x)
            outtake.retract()

        gripper.position = if (gamepad2.right_trigger > 0) 0.75 else 0.35

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
        private const val PRECISION_MODE_MULTIPLIER = 0.3
        private const val MOTOR_SPEED_MULTIPLIER = 0.75
        private const val INTAKE_POWER = 1.0
        private const val LIFT_POWER = 0.5

        private const val MAX_LIFT_HEIGHT_TICKS = 1200
    }
}
