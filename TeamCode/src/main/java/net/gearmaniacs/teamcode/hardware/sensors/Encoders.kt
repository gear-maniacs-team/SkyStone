package net.gearmaniacs.teamcode.hardware.sensors

import android.util.Log
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotor.RunMode
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.HardwareMap
import net.gearmaniacs.teamcode.RobotPos
import net.gearmaniacs.teamcode.TeamRobot
import net.gearmaniacs.teamcode.utils.IHardware
import net.gearmaniacs.teamcode.utils.IUpdatable
import net.gearmaniacs.teamcode.utils.PerformanceProfiler
import net.gearmaniacs.teamcode.utils.epsilonEquals
import kotlin.math.cos
import kotlin.math.sin

class Encoders : IHardware, IUpdatable {

    lateinit var left: DcMotor
        private set
    lateinit var right: DcMotor
        private set
    lateinit var back: DcMotor
        private set

    private var previousLeftPosition = 0.0
    private var previousRightPosition = 0.0
    private var previousBackPosition = 0.0

    override fun init(hardwareMap: HardwareMap) {
        val dcMotors = hardwareMap.dcMotor
        left = dcMotors["BL"]
        right = dcMotors["TR"]
        back = dcMotors["TL"]

        left.direction = DcMotorSimple.Direction.FORWARD
        right.direction = DcMotorSimple.Direction.FORWARD
        back.direction = DcMotorSimple.Direction.FORWARD

        setModeAll(RunMode.STOP_AND_RESET_ENCODER)
        setModeAll(RunMode.RUN_WITHOUT_ENCODER)

        initNative()
    }

    fun setModeAll(mode: RunMode) {
        left.mode = mode
        right.mode = mode
        back.mode = mode
    }

    private external fun initNative()

    private fun linearUpdate(leftPos: Double, rightPos: Double, backPos: Double) {
        val deltaBack = ticksToCM(backPos - previousBackPosition)
        val deltaRight = ticksToCM(rightPos - previousRightPosition)
        val deltaLeft = ticksToCM(leftPos - previousLeftPosition)

        val deltaAngle = (deltaLeft - deltaRight) / DISTANCE_BETWEEN_ENCODER_WHEELS

        val distance = (deltaLeft + deltaRight) / 2

        RobotPos.currentX += distance * cos(RobotPos.currentAngle) - deltaBack * sin(RobotPos.currentAngle)
        RobotPos.currentY += distance * sin(RobotPos.currentAngle) + deltaBack * cos(RobotPos.currentAngle)
        RobotPos.currentAngle += deltaAngle

        previousBackPosition = backPos
        previousLeftPosition = leftPos
        previousRightPosition = rightPos
    }

    private fun updateUsingArcs(leftPos: Double, rightPos: Double, backPos: Double) {
        val currentAngle = RobotPos.currentAngle

        val deltaBack = ticksToCM(backPos - previousBackPosition)
        val deltaRight = ticksToCM(rightPos - previousRightPosition)
        val deltaLeft = ticksToCM(leftPos - previousLeftPosition)

        val deltaAngle = (deltaLeft - deltaRight) / DISTANCE_BETWEEN_ENCODER_WHEELS

        val newX: Double
        val newY: Double
        if (deltaAngle epsilonEquals 0.0) {
            newX = deltaBack
            newY = deltaRight
        } else {
            newX = 2 * sin(deltaAngle / 2) * (deltaBack / deltaAngle + DISTANCE_BETWEEN_BACK_ENCODER_AND_CENTER)
            newY = 2 * sin(deltaAngle / 2) * (deltaRight / deltaAngle + DISTANCE_BETWEEN_ENCODER_WHEELS / 2)
        }

        val averageOrientation = -(currentAngle + deltaAngle / 2)

        // Calculate and update the position values
        // Rotate the cartesian coordinate system by transforming into polar form, adding the angle and then
        // transforming back into cartesian form.
        RobotPos.currentX += newX * cos(averageOrientation) - newY * sin(averageOrientation)
        RobotPos.currentY += newX * sin(averageOrientation) + newY * cos(averageOrientation)
        RobotPos.currentAngle += deltaAngle

        previousBackPosition = backPos
        previousLeftPosition = leftPos
        previousRightPosition = rightPos
    }

    /**
     * Calling updateNative before initNative will result in undefined behavior
     */
    private external fun updateNative(leftPos: Double, rightPos: Double, backPos: Double, currentAngle: Double): Result

    override fun update() {
        val leftPos: Double
        val rightPos: Double
        val backPos: Double

        val robot = TeamRobot.getRobot() ?: return
        if (!robot.isOpModeActive) return

        if (robot.useBulkRead) {
            leftPos = robot.bulkData2.getMotorCurrentPosition(left).toDouble()
            rightPos = -robot.bulkData1.getMotorCurrentPosition(right).toDouble()
            backPos = -robot.bulkData2.getMotorCurrentPosition(back).toDouble()
        } else {
            // All must return a positive value when moving forward
            leftPos = left.currentPosition.toDouble()
            rightPos = -right.currentPosition.toDouble()
            backPos = -back.currentPosition.toDouble()
        }

//        linearUpdate(leftPos, rightPos, backPos)
//        updateUsingArcs(leftPos, rightPos, backPos)

        val result = updateNative(ticksToCM(leftPos), ticksToCM(rightPos), ticksToCM(backPos), RobotPos.currentAngle)
        RobotPos.currentX += result.deltaX
        RobotPos.currentY += result.deltaY
        RobotPos.currentAngle += result.deltaAngle
    }

    class Result(
        val deltaX: Double,
        val deltaY: Double,
        val deltaAngle: Double
    )

    companion object {
        private const val DIAMETER = 7.2
        private const val TICKS_PER_REVOLUTION = 8192

        private const val DISTANCE_BETWEEN_ENCODER_WHEELS = 19.6125
        private const val DISTANCE_BETWEEN_BACK_ENCODER_AND_CENTER = 14.2 // The distance to the tracking center

        fun ticksToCM(x: Double) = (DIAMETER * Math.PI * x) / TICKS_PER_REVOLUTION
    }
}
