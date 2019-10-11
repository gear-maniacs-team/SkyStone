package org.firstinspires.ftc.teamcode

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorSimple
import org.firstinspires.ftc.robotcore.external.tfod.Recognition
import java.util.concurrent.Executors

@Autonomous(name = "CameraTest")
class CameraTest : OpMode() {


    private companion object {
        const val HEIGHT_COEFFICIENT = 20 * 271
        const val TICKS_PER_CM = 35.6507
        fun getCM(x: Float): Double = x * TICKS_PER_CM
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
    }

    override fun start() {
        isActive = true
        executor.submit {
            while (isActive) {
                cube = robot.vuforiaManager.getUpdatedRecognitions().firstOrNull { it.label == VuforiaManager.LABEL_GOLD_MINERAL }
                cube?.let {
                    telemetry.addData("Cube Height", it.height)
                    telemetry.addData("Left", it.left)
                    telemetry.addData("Dist", getDist(it.height))
                    val pos = (it.left + it.width / 2) / it.imageWidth
                    telemetry.addData("Pos", pos)
                    telemetry.update()
                }
                Thread.sleep(10)
            }
        }

        robot.wheels.setTargetPositionAll(0)
        robot.wheels.setModeAll(DcMotor.RunMode.RUN_TO_POSITION)
        robot.vuforiaManager.waitForDetector()
    }

    override fun loop() {
        val wheels = robot.wheels
        val cube = cube
        if (cube == null) {
            wheels.setPowerAll(0.0)
            return
        }

        val pos = (cube.left + cube.width / 2) / cube.imageWidth
        val dist = getDist(cube.height)

        if (pos > 0.45 && pos < 0.55) {
            // The cube is the middle
            move(dist, dist)
        } else if (pos < 0.45) {

        } else if (pos > 0.55) {

        }


        //moveTowards(0, 0)
    }

    override fun stop() {
        isActive = false
        executor.shutdown()
    }

    private fun move(leftDist: Float, rightRight: Float) {
        val wheels = robot.wheels

        wheels.left.targetPosition = wheels.left.currentPosition + getCM(leftDist).toInt()
        wheels.right.targetPosition = wheels.right.currentPosition + getCM(rightRight).toInt()

        telemetry.addData("Left", wheels.left.currentPosition)
        telemetry.addData("Left Target", getCM(leftDist))
        telemetry.addData("Right", wheels.right.currentPosition)

        wheels.setPowerAll(0.35)

        while (wheels.left.isBusy && wheels.right.isBusy) {
            telemetry.addData("Left", wheels.left.currentPosition)
            telemetry.addData("Right", wheels.right.currentPosition)
            //telemetry.update()
        }
    }

    private fun getDist(height: Float) = HEIGHT_COEFFICIENT / height
}