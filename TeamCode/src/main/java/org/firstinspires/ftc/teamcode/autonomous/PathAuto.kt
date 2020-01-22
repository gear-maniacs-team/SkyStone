package org.firstinspires.ftc.teamcode.autonomous

import android.util.Log
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.hardware.Servo
import org.firstinspires.ftc.teamcode.RobotPos
import org.firstinspires.ftc.teamcode.TeamRobot
import org.firstinspires.ftc.teamcode.motors.Intake
import org.firstinspires.ftc.teamcode.motors.Wheels
import org.firstinspires.ftc.teamcode.pid.PidController
import org.firstinspires.ftc.teamcode.sensors.Encoder
import org.firstinspires.ftc.teamcode.utils.*
import kotlin.math.abs
import kotlin.math.atan2
import kotlin.math.hypot
import kotlin.math.sin

@Autonomous(name = "Path")
open class PathAuto : LinearOpMode() {

    private val robot = TeamRobot()
    private val wheels = Wheels()
    private val encoder = Encoder()
    private val intake = Intake()

    private val xPid = PidController(0.6, 0.0000, 0.00)
    private val yPid = PidController(0.6, 0.0000, 0.00)
    private val rotationPid = PidController(2.0, 0.0, 0.0005)

    private lateinit var foundationLeft: Servo
    private lateinit var foundationRight: Servo

    //private val path by fastLazy { PathParser.parseAsset(hardwareMap.appContext, "path/test.json") }
    private val path = listOf(
        PathPoint(cmX=-37.44991801337282, cmY=52.241515486312025, angle=0.0),
        PathPoint(cmX=-37.91579419505235, cmY=59.80242770163131, angle=0.582572466564049),
        PathPoint(cmX=-22.86392497952157, cmY=83.85140117076618, angle=0.5637071426105487),
        PathPoint(cmX=-36.012792117491486, cmY=64.08201405799963, angle=0.5749700225827877),
        PathPoint(cmX=-35.715093894737535, cmY=61.06394786855772, angle=1.5557556891653017),
        PathPoint(cmX=-212.24789210975985, cmY=63.78807906913377, angle=1.6060162910414195),
        PathPoint(cmX=-212.93197927071836, cmY=54.99152481906848, angle=1.6807032638573258),
        PathPoint(cmX=-215.71217552134993, cmY=53.488314681181805, angle=3.1229854373023267),
        PathPoint(cmX=-215.62413808655725, cmY=66.82534951356647, angle=3.130587881283591)
    )

    override fun runOpMode() {
        robot.init(hardwareMap, listOf(wheels, intake, encoder), listOf(encoder))
        RobotPos.resetAll()

        xPid.setOutputRange(-0.7, 0.7)
        yPid.setOutputRange(-0.7, 0.7)
        rotationPid.setOutputRange(-0.5, 0.5)

        foundationLeft = hardwareMap.getDevice("foundation_left")
        foundationRight = hardwareMap.getDevice("foundation_right")

        while (!isStarted) {
            telemetry.addData("Status", "Waiting for start")
            telemetry.addData("Path Size", path.size)
            telemetry.update()
        }
        robot.start()

        actionIntake(true)

        path.forEach {
            RobotPos.targetX = it.cmX
            RobotPos.targetY = it.cmY
            RobotPos.targetAngle = it.angle

            goToPoint()

            performAction(it.action)
        }

        intake.left.power = 0.0
        intake.right.power = -0.0

        robot.stop()
    }

    private fun goToPoint() {
        xPid.setPoint = RobotPos.targetX
        yPid.setPoint = RobotPos.targetY
        rotationPid.setPoint = RobotPos.targetAngle

        xPid.reset()
        yPid.reset()
        rotationPid.reset()

        var count = 0
        val startTime = System.currentTimeMillis()

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

            if (!movement(x, y) && count > 5)
                break
            if (x smaller 0.2 && y smaller 0.2 && count > 5)
                break

            if (System.currentTimeMillis() - startTime > 3000)
                break

            Thread.sleep(10)
            count++
        }
    }

    private fun rotate(target: Double) {
        val rotation = rotationPid.compute(RobotPos.currentAngle)

        with(wheels) {
            rightFront.power = rotation
            leftFront.power = rotation
            rightBack.power = rotation
            leftBack.power = rotation
        }
    }

    private fun movement(x: Double, y: Double): Boolean {
        val magnitude = hypot(x, y) * MOTOR_SPEED_MULTIPLIER

        if (magnitude smaller 0.1) return false

        val angle = atan2(y, x) - Math.PI / 2

        val speedX = magnitude * sin(angle + Math.PI / 4)
        val speedY = magnitude * sin(angle - Math.PI / 4)

        with(wheels) {
            rightFront.power = -speedX
            leftFront.power = -speedY
            rightBack.power = speedY
            leftBack.power = speedX
        }

        return true
    }

    private fun performAction(action: Int) {
        when(action) {
            PathPoint.ACTION_NONE -> return
            PathPoint.ACTION_START_INTAKE -> actionIntake(true)
            PathPoint.ACTION_STOP_INTAKE -> actionIntake(false)
            PathPoint.ACTION_ATTACH_FOUNDATION -> foundation(true)
            PathPoint.ACTION_DETACH_FOUNDATION -> foundation(false)
            else -> Log.e(TAG, "Unsupported Action!")
        }
    }

    private fun actionIntake(start: Boolean) {
        val power = if (start) 1.0 else 0.0
        intake.left.power = power
        intake.right.power = -power
    }

    private fun foundation(attach: Boolean) {
        foundationLeft.position = if (attach) 0.0 else 1.0
        foundationRight.position = if (attach) 1.0 else 0.0
    }

    private companion object {
        const val TAG = "PathAuto"
        const val MOTOR_SPEED_MULTIPLIER = 0.6

        private infix fun Double.smaller(other: Double) = abs(this) < other
    }
}
