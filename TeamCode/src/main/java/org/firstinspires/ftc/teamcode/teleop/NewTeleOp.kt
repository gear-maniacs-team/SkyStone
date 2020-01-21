package org.firstinspires.ftc.teamcode.teleop

import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DcMotor
import org.firstinspires.ftc.teamcode.RobotPos
import org.firstinspires.ftc.teamcode.TeamRobot
import org.firstinspires.ftc.teamcode.motors.Intake
import org.firstinspires.ftc.teamcode.motors.Lift
import org.firstinspires.ftc.teamcode.motors.Wheels
import org.firstinspires.ftc.teamcode.sensors.Encoder
import org.firstinspires.ftc.teamcode.utils.MathUtils
import org.firstinspires.ftc.teamcode.utils.PerformanceProfiler
import kotlin.math.abs
import kotlin.math.atan2
import kotlin.math.hypot
import kotlin.math.sin

@TeleOp(name = "Mark TeleOp", group = "Good")
class NewTeleOp : OpMode() {

    private val performanceProfiler = PerformanceProfiler()
    private val robot = TeamRobot()
    private val wheels = Wheels()
    private val intake = Intake()
    private val encoder = Encoder()
    private val lift = Lift()

    private var liftTargetPosition = 0
    private var precisionModeOn = false
    private var rightFrontPower = 0.0
    private var leftFrontPower = 0.0
    private var rightBackPower = 0.0
    private var leftBackPower = 0.0

    private var resetAngle = 0.0
    private var orientationIndependentDrive = false

    override fun init() {
        robot.init(hardwareMap, listOf(wheels, encoder, intake, lift), listOf(encoder))
        wheels.setModeAll(DcMotor.RunMode.RUN_WITHOUT_ENCODER)

        robot.showPerformance(telemetry)
    }

    override fun start() {
        robot.start()
        RobotPos.resetAll()
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

        lift()
        intake()

        with(telemetry) {
            addData("X", RobotPos.currentX)
            addData("Y", RobotPos.currentY)
            addData("Heading", Math.toDegrees(MathUtils.angleWrap(RobotPos.currentAngle)))

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

        rightFrontPower += speedRight
        leftFrontPower += speedLeft
        rightBackPower += speedRight
        leftBackPower += speedLeft
    }

    private fun planeMovement() {
        val x = gamepad1.left_stick_x.toDouble()
        val y = -gamepad1.left_stick_y.toDouble()

        if (abs(x) == 0.0 && abs(y) == 0.0) return

        val magnitude = hypot(x, y) * if (precisionModeOn) PRECISION_MODE_MULTIPLIER else MOTOR_SPEED_MULTIPLIER

        val independentAngleCorrection = if (orientationIndependentDrive) RobotPos.currentAngle - resetAngle else 0.0
        val angle = atan2(y, x) - Math.PI / 2 - independentAngleCorrection
        val speedX = magnitude * sin(angle + Math.PI / 4)
        val speedY = magnitude * sin(angle - Math.PI / 4)

        rightFrontPower += -speedX
        leftFrontPower += -speedY
        rightBackPower += speedY
        leftBackPower += speedX
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
            gamepad2.dpad_down -> -100
            gamepad2.dpad_up -> 100
            else -> 0
        }

        liftTargetPosition += posChange
        if (posChange != 0)
            Thread.sleep(200)

        val power = if (liftTargetPosition == 0 && lift.left.currentPosition < 300) 0.0 else LIFT_POWER

        with(lift) {
            left.targetPosition = liftTargetPosition
            right.targetPosition = -liftTargetPosition
            left.power = power
            right.power = power
        }

        telemetry.addData("Lift Target Position", liftTargetPosition)
    }

    private companion object {
        private const val PRECISION_MODE_MULTIPLIER = 0.4
        private const val MOTOR_SPEED_MULTIPLIER = 0.7
        private const val INTAKE_POWER = 0.8
        private const val LIFT_POWER = 0.5
    }
}
