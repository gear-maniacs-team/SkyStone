package org.firstinspires.ftc.teamcode

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorSimple
import org.firstinspires.ftc.robotcore.external.tfod.Recognition
import org.firstinspires.ftc.teamcode.motors.SimpleMotors
import org.firstinspires.ftc.teamcode.utils.fastLazy
import java.util.concurrent.Executors

@Autonomous(name = "StoneFollower")
class StoneFollower : OpMode() {

    private companion object {
        const val DEFAULT_POWER = 0.7
        const val HEIGHT_COEFFICIENT = 20 * 271
        const val TICKS_PER_CM = 35.6507

        /*
         * Transforms CM into encoder ticks for the Hex HD 1:40 motor
         */
        fun getTicksFromCM(cm: Float): Double = cm * TICKS_PER_CM

        fun getDist(height: Float) = HEIGHT_COEFFICIENT / height

        /*
         * Compares two encoder positions with a margin of error of one digit
         */
        infix fun Int.isPositionEqual(other: Int) = this / 10 == other / 10

        fun mapToRange(inputStart: Double, inputEnd: Double, outputStart: Double, outputEnd: Double, input: Double) =
            outputStart + ((outputEnd - outputStart) / (inputEnd - inputStart)) * (input - inputStart)
    }

    private val robot = TeamRobot()
    private val executor = Executors.newSingleThreadExecutor()
    private var cube: Recognition? = null
    private val wheels by fastLazy { SimpleMotors(hardwareMap.dcMotor) }

    override fun init() {
        robot.init(hardwareMap)
        robot.vuforia.startDetectorAsync(hardwareMap)

        with(wheels) {
            left.direction = DcMotorSimple.Direction.REVERSE
            setModeAll(DcMotor.RunMode.STOP_AND_RESET_ENCODER)
            setModeAll(DcMotor.RunMode.RUN_USING_ENCODER)
        }
    }

    override fun start() {
        executor.submit {
            while (robot.isOpModeActive)
                cube = robot.vuforia.recognitions.firstOrNull { it.label == VuforiaManager.LABEL_GOLD_MINERAL }
            Thread.sleep(20)
        }

        robot.vuforia.waitForDetector()
    }

    override fun loop() {
        val cube = cube

        if (cube == null) {
            wheels.setPowerAll(0.0)
            return
        }

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
            else -> move(dist, DEFAULT_POWER, DEFAULT_POWER) // Middle
        }
    }

    override fun stop() {
        robot.stop()
        executor.shutdown()
    }

    private fun move(dist: Float, leftPower: Double, rightPower: Double) {
        val finalLeftPower = run {
            val currentPosition = wheels.left.currentPosition
            val targetPosition = currentPosition + getTicksFromCM(dist).toInt()

            if (!(currentPosition isPositionEqual targetPosition)) { // The motor hasn't reached its destination
                // Check if we need to go forwards or backwards
                val sign = if (currentPosition < targetPosition) 1 else -1

                leftPower * sign
            } else 0.0
        }

        val finalRightPower = run {
            val currentPosition = wheels.right.currentPosition
            val targetPosition = currentPosition + getTicksFromCM(dist).toInt()

            if (!(currentPosition isPositionEqual targetPosition)) { // The motor hasn't reached its destination
                // Check if we need to go forwards or backwards
                val sign = if (currentPosition < targetPosition) 1 else -1

                rightPower * sign
            } else 0.0
        }

        wheels.left.power = finalLeftPower
        wheels.right.power = finalRightPower

        telemetry.addData("Left", wheels.left.currentPosition)
        telemetry.addData("Right", wheels.right.currentPosition)
        telemetry.addData("Distance", getTicksFromCM(dist))
        telemetry.update()
    }
}