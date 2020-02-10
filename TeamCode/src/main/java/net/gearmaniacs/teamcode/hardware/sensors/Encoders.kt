package net.gearmaniacs.teamcode.hardware.sensors

import com.qualcomm.robotcore.hardware.DcMotor.RunMode
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.HardwareMap
import net.gearmaniacs.teamcode.RobotPos
import net.gearmaniacs.teamcode.TeamRobot
import net.gearmaniacs.teamcode.utils.*
import kotlin.math.cos
import kotlin.math.sin

class Encoders : IHardware, IUpdatable {

    lateinit var left: DcMotorEx
        private set
    lateinit var right: DcMotorEx
        private set
    lateinit var back: DcMotorEx
        private set

    private var previousLeftPosition = 0.0
    private var previousRightPosition = 0.0
    private var previousBackPosition = 0.0

    override fun init(hardwareMap: HardwareMap) {
        left = hardwareMap.getDevice("intake_left")
        right = hardwareMap.getDevice("intake_right")
        back = hardwareMap.getDevice("lift_left")

        left.direction = DcMotorSimple.Direction.FORWARD
        right.direction = DcMotorSimple.Direction.FORWARD
        back.direction = DcMotorSimple.Direction.FORWARD

        setModeAll(RunMode.STOP_AND_RESET_ENCODER)
        setModeAll(RunMode.RUN_WITHOUT_ENCODER)
    }

    fun setModeAll(mode: RunMode) {
        left.mode = mode
        right.mode = mode
        back.mode = mode
    }

//    private external fun initNative()

    private fun linearUpdate(leftPos: Double, rightPos: Double, backPos: Double) {
        val deltaBack = toCm(backPos - previousBackPosition)
        val deltaRight = toCm(rightPos - previousRightPosition)
        val deltaLeft = toCm(leftPos - previousLeftPosition)

        val deltaAngle = (deltaLeft - deltaRight) / DISTANCE_BETWEEN_ENCODER_WHEELS

        val distance = (deltaLeft + deltaRight) / 2

        RobotPos.currentX += distance * cos(RobotPos.currentAngle) - deltaBack * sin(RobotPos.currentAngle)
        RobotPos.currentY += distance * sin(RobotPos.currentAngle) + deltaBack * cos(RobotPos.currentAngle)
        RobotPos.currentAngle += deltaAngle

        previousBackPosition = backPos
        previousLeftPosition = leftPos
        previousRightPosition = rightPos
    }

    private fun arcUpdate(leftPos: Double, rightPos: Double, backPos: Double) {
        val currentAngle = RobotPos.currentAngle

        val deltaBack = toCm(backPos - previousBackPosition)
        val deltaRight = toCm(rightPos - previousRightPosition)
        val deltaLeft = toCm(leftPos - previousLeftPosition)

        val deltaAngle = (deltaLeft - deltaRight) / DISTANCE_BETWEEN_ENCODER_WHEELS

        var newX = deltaBack
        var newY = deltaRight
        if (!(deltaAngle epsilonEquals 0.0)) {
            val sinDeltaAngle = sin(deltaAngle / 2.0)
            newX = 2.0 * sinDeltaAngle * (deltaBack / deltaAngle + DISTANCE_BETWEEN_BACK_ENCODER_AND_CENTER)
            newY = 2.0 * sinDeltaAngle * (deltaRight / deltaAngle + DISTANCE_BETWEEN_ENCODER_WHEELS / 2)
        }

        val averageOrientation = -(currentAngle + deltaAngle / 2.0)

        // Calculate and update the position values
        // Rotate the cartesian coordinate system by transforming into polar form, adding the angle and then
        // transforming back into cartesian form.
        val sinAverageOrientation = sin(averageOrientation)
        val cosAverageOrientation = cos(averageOrientation)
        RobotPos.currentX += newX * cosAverageOrientation - newY * sinAverageOrientation
        RobotPos.currentY += newX * sinAverageOrientation + newY * cosAverageOrientation
        RobotPos.currentAngle += deltaAngle

        previousBackPosition = backPos
        previousLeftPosition = leftPos
        previousRightPosition = rightPos
    }

    /**
     * Calling updateNative before initNative will result in undefined behavior
     */
//    private external fun updateNative(leftPos: Double, rightPos: Double, backPos: Double)

    override fun update() {
        val robot = TeamRobot.getRobot()
        if (robot == null || !robot.isOpModeActive) return

        val leftPos = -left.getCurrentPosition(robot.bulkData2).toDouble()
        val rightPos = right.getCurrentPosition(robot.bulkData1).toDouble()
        val backPos = back.getCurrentPosition(robot.bulkData2).toDouble()

//        linearUpdate(leftPos, rightPos, backPos)
        arcUpdate(leftPos, rightPos, backPos)
    }

    companion object {
        private const val M_DIAMETER = 7.2
        private const val M_TICKS_PER_REVOLUTION = 8192

        private const val DISTANCE_BETWEEN_ENCODER_WHEELS = 19.6125
        private const val DISTANCE_BETWEEN_BACK_ENCODER_AND_CENTER = 14.2 // The distance to the tracking center

        fun toCm(ticks: Double) = (ticks * M_DIAMETER * Math.PI) / M_TICKS_PER_REVOLUTION
    }
}
