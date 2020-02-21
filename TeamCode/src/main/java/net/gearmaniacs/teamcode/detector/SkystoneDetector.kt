package net.gearmaniacs.teamcode.detector

import org.firstinspires.ftc.robotcore.external.Telemetry
import org.opencv.core.*
import org.opencv.imgproc.Imgproc
import org.openftc.easyopencv.OpenCvPipeline
import java.util.*

class SkystoneDetector : OpenCvPipeline {
    enum class SkystonePosition {
        LEFT_STONE, CENTER_STONE, RIGHT_STONE
    }q

    // These are the mats we need, I will be explaining them as we go
    private val matYCrCb = Mat()
    private val means = arrayListOf<Scalar>()
    private var firstStonePosition = 0.0
    private var secondStonePosition = 0.0
    private var thirdStonePosition = 0.0
    private var firstSkystonePercentage = 0.0
    private var percentSpacing = 0.0
    private var stoneWidth = 0.0
    private var stoneHeight = 0.0
    private var defaultValues: Boolean
    private var spacing = 0.0
    private var tl: Telemetry? = null
    @Volatile
    var skystonePosition: SkystonePosition? = null
        private set
    val blocks = arrayListOf<Rect>()

    @JvmOverloads
    constructor(tl: Telemetry? = null) {
        this.tl = tl
        defaultValues = true
    }

    @JvmOverloads
    constructor(
        firstSkystonePositionPercentage: Double,
        percentSpacing: Double,
        stoneWidth: Double,
        stoneHeight: Double,
        tl: Telemetry? = null
    ) {
        defaultValues = false
        firstSkystonePercentage = firstSkystonePositionPercentage
        this.percentSpacing = percentSpacing
        this.stoneWidth = stoneWidth
        this.stoneHeight = stoneHeight
        this.tl = tl
        skystonePosition = null
    }
    //These will be the points for our rectangle
    /**
     * This will create the rectangles
     *
     * @param frame     the input mat
     * @param rect      the rectangle
     * @param color     the color of the rectangle when it is displayed on screen
     * @param thickness the thickness of the rectangle
     */
    private fun drawRectangle(frame: Mat, rect: Rect, color: Scalar, thickness: Int): Mat {
        Imgproc.rectangle(frame, rect, color, thickness)
        //submat simply put is cropping the mat
        return frame.submat(rect)
    }

    override fun processFrame(input: Mat): Mat {
        setValues(input.width().toDouble(), input.height().toDouble())
        try {
            /**
             * input which is in RGB is the frame the camera gives
             * We convert the input frame to the color space matYCrCb
             * Then we store this converted color space in the mat matYCrCb
             * For all the color spaces go to
             * https://docs.opencv.org/3.4/d8/d01/group__imgproc__color__conversions.html
             */
            Imgproc.cvtColor(input, matYCrCb, Imgproc.COLOR_RGB2YCrCb)
            for (stone in blocks!!) {
                val currentMat = Mat()
                Core.extractChannel(drawRectangle(matYCrCb, stone, Scalar(255.0, 0.0, 255.0), 2), currentMat, 2)
                means.add(Core.mean(currentMat))
                currentMat.release()
            }
            var max = means[0]
            var biggestIndex = 0
            for (k in means) {
                if (k.`val`[0] > max.`val`[0]) {
                    max = k
                    biggestIndex = means.indexOf(k)
                }
            }
            when (biggestIndex) {
                0 -> {
                    skystonePosition = SkystonePosition.LEFT_STONE
                    Imgproc.rectangle(input, blocks!![0], Scalar(0.0, 255.0, 0.0), 30)
                    Imgproc.rectangle(input, blocks!![1], Scalar(255.0, 0.0, 0.0), 30)
                    Imgproc.rectangle(input, blocks!![2], Scalar(255.0, 0.0, 0.0), 30)
                }
                1 -> {
                    skystonePosition = SkystonePosition.CENTER_STONE
                    Imgproc.rectangle(input, blocks!![0], Scalar(255.0, 0.0, 0.0), 30)
                    Imgproc.rectangle(input, blocks!![1], Scalar(0.0, 255.0, 0.0), 30)
                    Imgproc.rectangle(input, blocks!![2], Scalar(255.0, 0.0, 0.0), 30)
                }
                2 -> {
                    Imgproc.rectangle(input, blocks!![0], Scalar(255.0, 0.0, 0.0), 30)
                    Imgproc.rectangle(input, blocks!![1], Scalar(255.0, 0.0, 0.0), 30)
                    Imgproc.rectangle(input, blocks!![2], Scalar(0.0, 255.0, 0.0), 30)
                    skystonePosition = SkystonePosition.RIGHT_STONE
                }
                else -> {
                    skystonePosition = SkystonePosition.CENTER_STONE
                    Imgproc.rectangle(input, blocks!![0], Scalar(255.0, 0.0, 0.0), 30)
                    Imgproc.rectangle(input, blocks!![1], Scalar(255.0, 0.0, 0.0), 30)
                    Imgproc.rectangle(input, blocks!![2], Scalar(255.0, 0.0, 0.0), 30)
                }
            }
            if (tl != null) {
                tl!!.addData("Skystone Position", skystonePosition)
                tl!!.update()
            }
            means.clear()
        } catch (e: Exception) {
            if (tl != null) {
                tl!!.addData("Exception", e)
                tl!!.update()
            }
        }
        return input
    }

    /**
     * Sets the target rectangles only once using input's width and height.
     *
     * @param width  Width of Frame
     * @param height Height of Frame
     */
    private fun setValues(width: Double, height: Double) {
        if (blocks.isEmpty()) {
            if (defaultValues) { // Set default values
                firstSkystonePercentage = 25.0
                percentSpacing = 25.0
                stoneHeight = 50.0
                stoneWidth = 50.0
            }
            spacing = percentSpacing * width / 100
            firstStonePosition = firstSkystonePercentage / 100 * width
            secondStonePosition = firstStonePosition + spacing
            thirdStonePosition = secondStonePosition + spacing
            blocks.add(
                Rect(
                    Point(firstStonePosition - stoneWidth / 2, 0.50 * height - stoneHeight / 2),
                    Point(firstStonePosition + stoneWidth / 2, 0.50 * height + stoneHeight / 2)
                )
            )
            blocks.add(
                Rect(
                    Point(secondStonePosition - stoneWidth / 2, 0.50 * height - stoneHeight / 2),
                    Point(secondStonePosition + stoneWidth / 2, 0.50 * height + stoneHeight / 2)
                )
            )
            blocks.add(
                Rect(
                    Point(thirdStonePosition - stoneWidth / 2, 0.50 * height - stoneHeight / 2),
                    Point(thirdStonePosition + stoneWidth / 2, 0.50 * height + stoneHeight / 2)
                )
            )
        }
    }
}
