package net.gearmaniacs.teamcode.detector

import com.disnodeteam.dogecv.DogeCV
import com.disnodeteam.dogecv.detectors.DogeCVDetector
import com.disnodeteam.dogecv.filters.LeviColorFilter
import com.disnodeteam.dogecv.scoring.MaxAreaScorer
import com.disnodeteam.dogecv.scoring.PerfectAreaScorer
import com.disnodeteam.dogecv.scoring.RatioScorer
import org.opencv.core.*
import org.opencv.imgproc.Imgproc
import java.util.*

class StoneDetector : DogeCVDetector() {

    var areaScoringMethod = DogeCV.AreaScoringMethod.MAX_AREA // Setting to decide to use MaxAreaScorer or PerfectAreaScorer
    //Create the default filters and scorers
    var filter = LeviColorFilter(LeviColorFilter.ColorPreset.YELLOW, 70.0) //Default Yellow blackFilter

    var stonesToFind = 2

    var ratioScorerForShortFace = RatioScorer(1.25, 3.0) // Used to find the short face of the stone
    var ratioScorerForLongFace = RatioScorer(0.625, 3.0) // Used to find the long face of the stone
    var maxAreaScorer = MaxAreaScorer(5.0)                    // Used to find largest objects
    var perfectAreaScorer = PerfectAreaScorer(5000.0, 0.05) // Used to find objects near a tuned area value

    // This listener will be called each time after the frame has been processed
    var processedListener: ((screenPositions: List<Point>, rectangles: List<Rect>) -> Unit)? = null

    // Results of the detector
    private val screenPositions = ArrayList<Point>() // Screen positions of the stones
    private val foundRects = ArrayList<Rect>() // Found rect

    private val rawImage = Mat()
    private val workingMat = Mat()
    private val displayMat = Mat()
    private val yellowMask = Mat()
    private val hierarchy = Mat()

    // Cached objects to avoid unnecessary allocations
    private val contoursYellow = ArrayList<MatOfPoint>()
    private val matPointsComparator = Comparator<MatOfPoint> { matOfPoint, t1 ->
        if (calculateScore(matOfPoint) > calculateScore(t1)) 1 else 0
    }

    init {
        detectorName = "Stone Detector"
    }

    fun foundScreenPositions() = screenPositions.toList()

    fun foundRectangles() = foundRects.toList()

    override fun process(input: Mat): Mat {
        screenPositions.clear()
        foundRects.clear()

        input.copyTo(rawImage)
        input.copyTo(workingMat)
        input.copyTo(displayMat)
        input.copyTo(yellowMask)

        // Imgproc.GaussianBlur(workingMat,workingMat,new Size(5,5),0);
        filter.process(workingMat.clone(), yellowMask)

        contoursYellow.clear()
        Imgproc.findContours(yellowMask, contoursYellow, hierarchy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE)
        Imgproc.drawContours(displayMat, contoursYellow, -1, Scalar(230.0, 70.0, 70.0), 2)

        // Current result
        contoursYellow.sortWith(matPointsComparator)

        val subList: List<MatOfPoint> = if (contoursYellow.size > stonesToFind) {
            contoursYellow.take(stonesToFind)
        } else {
            contoursYellow
        }

        for (contour in subList) {
            val rect = Imgproc.boundingRect(contour)

            // Show chosen result
            Imgproc.rectangle(displayMat, rect.tl(), rect.br(), Scalar(255.0, 0.0, 0.0), 4)
            Imgproc.putText(displayMat, "Chosen", rect.tl(), 0, 1.0, Scalar(255.0, 255.0, 255.0))

            screenPositions.add(Point(rect.x.toDouble(), rect.y.toDouble()))
            foundRects.add(rect)
        }

        found = foundRects.size > 0

        processedListener?.invoke(screenPositions, foundRects)

        return when (stageToRenderToViewport) {
            Stage.THRESHOLD -> {
                Imgproc.cvtColor(yellowMask, yellowMask, Imgproc.COLOR_GRAY2BGR)
                yellowMask
            }
            Stage.RAW_IMAGE -> rawImage
            else -> displayMat
        }
    }

    override fun useDefaults() {
        addScorer(ratioScorerForShortFace)
        addScorer(ratioScorerForLongFace)

        // Add diffrent scoreres depending on the selected mode
        if (areaScoringMethod == DogeCV.AreaScoringMethod.MAX_AREA)
            addScorer(maxAreaScorer)

        if (areaScoringMethod == DogeCV.AreaScoringMethod.PERFECT_AREA)
            addScorer(perfectAreaScorer)
    }
}