package net.gearmaniacs.teamcode.autonomous.demo

import com.qualcomm.robotcore.eventloop.opmode.Disabled
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import net.gearmaniacs.teamcode.RobotPos
import net.gearmaniacs.teamcode.TeamRobot
import net.gearmaniacs.teamcode.hardware.motors.Wheels
import net.gearmaniacs.teamcode.pid.PidController
import net.gearmaniacs.teamcode.hardware.sensors.Encoders
import kotlin.math.abs
import kotlin.math.atan2
import kotlin.math.hypot
import kotlin.math.sin

@Disabled
@TeleOp(name = "StaiPăLoc")
class PidAutoTuner : LinearOpMode() {

    private val robot = TeamRobot()
    private val wheels = Wheels()
    private val encoder = Encoders()

    private val xPid = PidController(0.45, 0.0, 20.0)
    private val yPid = PidController(0.45, 0.0, 20.0)

    override fun runOpMode() {
        robot.init(hardwareMap, listOf(wheels, encoder), listOf(encoder))
        RobotPos.resetAll()

        xPid.setOutputRange(-MOTOR_SPEED, MOTOR_SPEED)
        yPid.setOutputRange(-MOTOR_SPEED, MOTOR_SPEED)

        while (!isStarted) {
            telemetry.addData("Status", "Waiting for start")
            telemetry.update()
        }
        robot.start()

        goToPoint()

        robot.stop()
    }

    private fun goToPoint() {
        xPid.reset()
        yPid.reset()

        while (opModeIsActive()) {
            val x = xPid.compute(RobotPos.currentX)
            val y = yPid.compute(RobotPos.currentY)

            with(telemetry) {
                addData("Current X", RobotPos.currentX)
                addData("Current Y", RobotPos.currentY)
                addData("Current Angle", RobotPos.currentAngle)
                addData("--", "--")
                addData("Target X", RobotPos.targetX)
                addData("Target Y", RobotPos.targetY)
                addData("Target Angle", RobotPos.targetAngle)
                addData("--", "--")
                addData("PID X", x)
                addData("PID Y", y)
                update()
            }

            movement(x, y)
            Thread.sleep(10)
        }
    }

    private fun strafe() {
        xPid.reset()

        while (opModeIsActive()) {
            val x = xPid.compute(RobotPos.currentX)

            with(telemetry) {
                addData("Current X", RobotPos.currentX)
                addData("Current Y", RobotPos.currentY)
                addData("Current Angle", RobotPos.currentAngle)
                addData("--", "--")
                addData("Target X", RobotPos.targetX)
                addData("Target Y", RobotPos.targetY)
                addData("Target Angle", RobotPos.targetAngle)
                addData("--", "--")
                addData("PID X", x)
                update()
            }

            movement(x, 0.0)
            Thread.sleep(5)
        }
    }

    private fun move() {
        yPid.reset()

        while (opModeIsActive()) {
            val y = yPid.compute(RobotPos.currentX)

            with(telemetry) {
                addData("Current X", RobotPos.currentX)
                addData("Current Y", RobotPos.currentY)
                addData("Current Angle", RobotPos.currentAngle)
                addData("--", "--")
                addData("Target X", RobotPos.targetX)
                addData("Target Y", RobotPos.targetY)
                addData("Target Angle", RobotPos.targetAngle)
                addData("--", "--")
                addData("PID Y", y)
                update()
            }

            movement(0.0, y)
            Thread.sleep(5)
        }
    }

    private fun movement(x: Double, y: Double): Boolean {
        val magnitude = hypot(x, y)

        if (magnitude smaller 0.1) return false

        val angle = atan2(y, x) - Math.PI / 2

        val speedX = magnitude * sin(angle + Math.PI / 4)
        val speedY = magnitude * sin(angle - Math.PI / 4)

        with(wheels) {
            rightFront.power = -speedX
            leftFront.power = -speedY
            rightRear.power = speedY
            leftRear.power = speedX
        }

        return true
    }

    private companion object {
        const val MOTOR_SPEED = 0.5
        
        private infix fun Double.smaller(other: Double) = abs(this) < other
    }
}
