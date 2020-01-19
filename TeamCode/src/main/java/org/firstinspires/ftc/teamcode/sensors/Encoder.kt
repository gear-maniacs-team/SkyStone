package org.firstinspires.ftc.teamcode.sensors

import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotor.RunMode
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.teamcode.RobotPos
import org.firstinspires.ftc.teamcode.utils.IHardware
import org.firstinspires.ftc.teamcode.utils.IUpdatable
import org.firstinspires.ftc.teamcode.utils.epsilonEquals
import kotlin.math.cos
import kotlin.math.sin

class Encoder : IHardware, IUpdatable {

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
        left = dcMotors["TL"]
        right = dcMotors["BR"]
        back = dcMotors["TR"]

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

    fun updateUsingArcs() {
        val currentAngle = RobotPos.currentAngle
        val leftPosition = left.currentPosition.toDouble()// Must return a positive value when moving forward
        val rightPosition = right.currentPosition.toDouble()
        val backPosition = back.currentPosition.toDouble()

        val leftChange = leftPosition - previousLeftPosition
        val rightChange = rightPosition - previousRightPosition
        val backChange = backPosition - previousBackPosition

        // Calculate Angle
        val changeInAngle = (leftChange - rightChange) / DISTANCE_BETWEEN_ENCODER_WHEELS

        // Get the components of the motion
        val newX: Double
        val newY: Double
        if (changeInAngle epsilonEquals 0.0) {
            newX = backChange
            newY = rightChange
        } else {
            newX = 2 * sin(currentAngle / 2) * (backChange / changeInAngle + DISTANCE_BETWEEN_BACK_ENCODER_AND_CENTER)
            newY = 2 * sin(currentAngle / 2) * (rightChange / changeInAngle + DISTANCE_BETWEEN_ENCODER_WHEELS / 2)
        }
        val averageOrientation = currentAngle + changeInAngle / 2

        // Calculate and update the position values
        // Rotate the cartesian coordinate system by transforming into polar form, adding the angle and then
        // transforming back into cartesian form.
        RobotPos.currentX += newX * cos(averageOrientation) + newY * sin(averageOrientation)
        RobotPos.currentY += -newX * sin(averageOrientation) + newY * cos(averageOrientation)
        RobotPos.currentAngle += changeInAngle

        previousLeftPosition = leftPosition
        previousRightPosition = rightPosition
        previousBackPosition = backPosition
    }

    fun updateUsingArcsPotentiallyActuallyWorking() {
        val currentAngle = RobotPos.currentAngle
        val backPos = back.currentPosition.toDouble()
        val leftPos = -left.currentPosition.toDouble() // Must return a positive value when moving forward
        val rightPos = right.currentPosition.toDouble()

        val deltaBack = backPos - previousBackPosition
        val deltaRight = rightPos - previousRightPosition
        val deltaLeft = leftPos - previousLeftPosition

        val wheelTravelLeft = ticksToCM(deltaLeft)
        val wheelTravelRight = ticksToCM(deltaRight)

        val deltaAngle = (wheelTravelLeft - wheelTravelRight) / DISTANCE_BETWEEN_ENCODER_WHEELS

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
        RobotPos.currentX += newX * cos(averageOrientation) + newY * sin(averageOrientation)
        RobotPos.currentY += -newX * sin(averageOrientation) + newY * cos(averageOrientation)
        RobotPos.currentAngle -= deltaAngle

        previousBackPosition = backPos
        previousLeftPosition = leftPos
        previousRightPosition = rightPos
    }

    override fun update() {
        updateUsingArcsPotentiallyActuallyWorking()
    }

    companion object {
        private const val DIAMETER = 7.2
        private const val TICKS_PER_REVOLUTION = 4096

        private const val DISTANCE_BETWEEN_ENCODER_WHEELS = 19.772
        private const val DISTANCE_BETWEEN_BACK_ENCODER_AND_CENTER = 14.2 // The distance to the tracking center

        fun ticksToCM(x: Double) = (DIAMETER * Math.PI * x) / TICKS_PER_REVOLUTION
    }
}
