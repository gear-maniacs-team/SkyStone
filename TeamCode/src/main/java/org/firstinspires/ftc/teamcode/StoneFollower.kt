package org.firstinspires.ftc.teamcode

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import org.firstinspires.ftc.robotcore.external.tfod.Recognition
import java.util.concurrent.Executors
import kotlin.time.ExperimentalTime

@Autonomous(name = "StoneFollower")
class StoneFollower : OpMode() {

    private companion object {
        const val DEFAULT_POWER = 0.7
        const val HEIGHT_COEFFICIENT = 20 * 271
        const val TICKS_PER_CM = 35.6507

        fun getTicksFromCM(x: Float): Double = x * TICKS_PER_CM

        fun getDist(height: Float) = HEIGHT_COEFFICIENT / height

        fun isPositionEqual(first: Int, second: Int) = first / 10 == second / 10

        fun mapToRange(inputStart: Double, inputEnd: Double, outputStart: Double, outputEnd: Double, input: Double) =
            outputStart + ((outputEnd - outputStart) / (inputEnd - inputStart)) * (input - inputStart)
    }

    private var isActive = false
    private val robot = TeamRobot()
    private val executor = Executors.newSingleThreadExecutor()
    private var cube: Recognition? = null

    override fun init() {
        robot.onInit(hardwareMap)
        val wheels = robot.wheels

        robot.vuforiaManager.startDetectorAsync(hardwareMap)
        wheels.left.direction = DcMotorSimple.Direction.REVERSE

        wheels.setModeAll(DcMotor.RunMode.STOP_AND_RESET_ENCODER)
        wheels.setModeAll(DcMotor.RunMode.RUN_USING_ENCODER)
    }

    override fun start() {
        isActive = true
        executor.submit {
            while (isActive) {
                cube = robot.vuforiaManager.recognitions.firstOrNull { it.label == VuforiaManager.LABEL_GOLD_MINERAL }
                cube?.let {
                    telemetry.addData("Cube Height", it.height)
                    telemetry.addData("Dist", getDist(it.height))
                    telemetry.update()
                }
            }
            Thread.sleep(20)
        }

        robot.vuforiaManager.waitForDetector()
    }

    override fun loop() {
        val wheels = robot.wheels
        val cube = cube

        if (cube == null) {
            wheels.setPowerAll(0.0)
            return
        }

        val leftPid = wheels.left as DcMotorEx
        leftPid.velocity

        val cubePos = (cube.left + cube.width / 2) / cube.imageWidth
        val dist = getDist(cube.height)

        when {
            cubePos < 0.47 -> { // Left
                val leftPower = mapToRange(0.05, 0.47, 0.5, 1.0, cubePos.toDouble())
                move(dist, DEFAULT_POWER * leftPower, DEFAULT_POWER)
            }
            cubePos > 0.53 -> { // Right
                val rightPower = mapToRange(0.53, 1.0, 0.5, 1.0, cubePos.toDouble())
                move(dist, DEFAULT_POWER, DEFAULT_POWER * rightPower)
            }
            else -> move(dist, DEFAULT_POWER, DEFAULT_POWER)
        }
    }

    override fun stop() {
        isActive = false
        executor.shutdown()
        robot.vuforiaManager.stopCamera()
    }

    private fun move(dist: Float, leftPower: Double, rightPower: Double) {
        val wheels = robot.wheels

        val finalLeftPower = run {
            val leftPos = wheels.left.currentPosition
            val leftTarget = leftPos + getTicksFromCM(dist).toInt()

            if (!isPositionEqual(leftPos, leftTarget)) {
                val sign = if (leftPos < leftTarget) 1 else -1

                leftPower * sign
            } else 0.0
        }

        val finalRightPower = run {
            val rightPos = wheels.right.currentPosition
            val rightTarget = rightPos + getTicksFromCM(dist).toInt()

            if (!isPositionEqual(rightPos, rightTarget)) {
                val sign = if (rightPos < rightTarget) 1 else -1

                rightPower * sign
            } else 0.0
        }

        wheels.left.power = finalLeftPower
        wheels.right.power = finalRightPower

        telemetry.addData("Left", wheels.left.currentPosition)
        telemetry.addData("Right", wheels.right.currentPosition)
        telemetry.addData("Distance", getTicksFromCM(dist))
    }
}