package org.firstinspires.ftc.teamcode.autonomous

import com.qualcomm.robotcore.eventloop.opmode.Disabled
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DistanceSensor
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit
import org.firstinspires.ftc.robotcore.external.tfod.Recognition
import org.firstinspires.ftc.teamcode.TeamRobot
import org.firstinspires.ftc.teamcode.detector.VuforiaManager
import org.firstinspires.ftc.teamcode.motors.Wheels
import org.firstinspires.ftc.teamcode.utils.Ranges
import java.util.concurrent.Executors

@TeleOp(name = "Aport")
class StoneFollowerMecanum : OpMode() {

    private companion object {
        const val DEFAULT_POWER = 0.7

        private const val MOTOR_TICKS = 1120
        private const val WHEELS_DIAMETER = 10.16
        private const val TICKS_PER_CM = MOTOR_TICKS / (WHEELS_DIAMETER * Math.PI)

        private const val HEIGHT_COEFFICIENT = 20 * 271

        /*
         * Transforms CM into encoder ticks
         */
        fun cmToTicks(cm: Double): Double = cm * TICKS_PER_CM

        fun getDist(height: Float) = HEIGHT_COEFFICIENT / height
    }

    private val robot = TeamRobot()
    private val wheels = Wheels()
    private lateinit var distanceSensor: DistanceSensor

    private val executor = Executors.newFixedThreadPool(2)
    private var cube: Recognition? = null
    private var distanceSensorUnit = 0.0

    override fun init() {
        robot.init(hardwareMap)
        wheels.init(hardwareMap)
        robot.vuforia.startDetectorAsync(hardwareMap)
        distanceSensor = hardwareMap.get(DistanceSensor::class.java, "cargo_distance")

        with(wheels) {
            setModeAll(DcMotor.RunMode.STOP_AND_RESET_ENCODER)
            setModeAll(DcMotor.RunMode.RUN_USING_ENCODER)
        }
    }

    override fun start() {
        executor.submit {
            while (robot.isOpModeActive) {
                robot.vuforia.waitForDetector()
                cube = robot.vuforia.recognitions.firstOrNull {
                    it.label == VuforiaManager.LABEL_STONE
                }
                Thread.sleep(100)
            }
        }

        executor.submit {
            while (robot.isOpModeActive) {
                distanceSensorUnit = distanceSensor.getDistance(DistanceUnit.CM)
                Thread.sleep(100)
            }
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
        val cameraDistance = getDist(cube.height).toDouble()

        val dist = if (cubePos < 0.47 || cubePos > 0.53) distanceSensorUnit else cameraDistance

        if (distanceSensorUnit == Double.MAX_VALUE) {
            wheels.setPowerAll(0.0)
            return
        }

        move(
            Ranges.map(0.01, 0.99, -0.5, 0.5, cubePos.toDouble()),
            Ranges.map(100.0, 2000.0, 0.0, DEFAULT_POWER, minOf(cmToTicks(dist), 2000.0))
        )

        telemetry.addData("Camera Distance", cameraDistance)
        telemetry.addData("Sensor Distance", distanceSensorUnit)
        telemetry.update()
    }

    private fun move(x: Double, y: Double) {
        var powerLeft = y + x
        var powerRight = -y + x

        // Find the biggest value
        val max = maxOf(powerLeft, powerRight)

        if (max > 1) {
            powerLeft /= max
            powerRight /= max
        }

        with(wheels) {
            rightFront.power = powerRight
            leftFront.power = powerLeft
            rightBack.power = powerRight
            leftBack.power = powerLeft
        }
    }
}
