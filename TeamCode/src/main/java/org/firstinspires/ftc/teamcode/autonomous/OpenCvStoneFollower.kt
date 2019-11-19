package org.firstinspires.ftc.teamcode.autonomous

import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DcMotor
import org.firstinspires.ftc.teamcode.TeamRobot
import org.firstinspires.ftc.teamcode.detector.OpenCvManager
import org.firstinspires.ftc.teamcode.utils.fastLazy
import org.opencv.core.Rect

@TeleOp(name = "AportCV")
class OpenCvStoneFollower : OpMode() {

    private companion object {
        const val DEFAULT_POWER = 0.5

        private const val MOTOR_TICKS = 1120
        private const val WHEELS_DIAMETER = 10.16
        private const val TICKS_PER_CM = MOTOR_TICKS / (WHEELS_DIAMETER * Math.PI)

        private const val HEIGHT_COEFFICIENT = 20 * 271

        const val CAMERA_WIDTH = 640
        const val CAMERA_HEIGHT = 480

        /*
         * Transforms CM into encoder ticks
         */
        fun cmToTicks(cm: Double): Double = cm * TICKS_PER_CM

        fun getDist(height: Float) = HEIGHT_COEFFICIENT / height

        fun mapToRange(inputStart: Double, inputEnd: Double, outputStart: Double, outputEnd: Double, input: Double) =
                outputStart + ((outputEnd - outputStart) / (inputEnd - inputStart)) * (input - inputStart)
    }

    private val robot = TeamRobot()
    private val wheelMotors by fastLazy { robot.wheelsMotors }
    private lateinit var detector: StoneDetector
    private lateinit var detectorManager: OpenCvManager

    @Volatile
    private var foundRectangle: Rect? = null

    override fun init() {
        robot.init(hardwareMap)

        detector = StoneDetector().apply {
            useDefaults()
            stonesToFind = 1
            processedListener = { _, rectangles: List<Rect> ->
                foundRectangle = rectangles.firstOrNull()
                require(rectangles.size < 2)
            }
        }

        detectorManager = OpenCvManager(detector).apply {
            startCamera(hardwareMap)
            startDetector(CAMERA_WIDTH, CAMERA_HEIGHT)
        }

        with(wheelMotors) {
            setModeAll(DcMotor.RunMode.STOP_AND_RESET_ENCODER)
            setModeAll(DcMotor.RunMode.RUN_USING_ENCODER)
        }
    }

    override fun loop() {
        val cube = foundRectangle

        if (cube == null) {
            wheelMotors.setPowerAll(0.0)
            telemetry.addData("Rect", "is null")
            return
        }

        telemetry.addData("Rect", cube.toString())

        val cubePos: Double = (cube.x.toDouble() + cube.width.toDouble() / 2) / CAMERA_WIDTH
        val cameraDistance = getDist(cube.height.toFloat()).toDouble()

        val dist = cameraDistance
        /*val dist = if (cubePos < 0.47 || cubePos > 0.53) distanceSensorUnit else cameraDistance

        if (distanceSensorUnit == Double.MAX_VALUE) {
            wheelMotors.setPowerAll(0.0)
            return
        }*/

        move(
            mapToRange(0.01, 0.99, -0.5, 0.5, cubePos.toDouble()),
            mapToRange(100.0, 2000.0, 0.0, DEFAULT_POWER, minOf(cmToTicks(dist), 2000.0))
        )

        telemetry.addData("Cube Pos", cubePos)
        telemetry.addData("Camera Distance", cameraDistance)
        //telemetry.addData("Sensor Distance", distanceSensorUnit)
        detectorManager.printTelemetry(telemetry)
        telemetry.update()
    }

    override fun stop() {
        detectorManager.hidePreview()
        detectorManager.stopDetector()
    }

    private fun move(x: Double, y: Double) {
        var powerLeft = y + x
        var powerRight = -y + x

        val max = maxOf(powerLeft, powerRight)

        if (max > 1) {
            powerLeft /= max
            powerRight /= max
        }

        wheelMotors.leftBack.power = powerLeft
        wheelMotors.leftFront.power = powerLeft
        wheelMotors.rightBack.power = powerRight
        wheelMotors.rightFront.power = powerRight
    }
}