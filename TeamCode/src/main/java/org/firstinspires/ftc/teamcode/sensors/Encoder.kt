package org.firstinspires.ftc.teamcode.sensors

import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotor.RunMode
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.teamcode.RobotPos
import org.firstinspires.ftc.teamcode.TeamRobot
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

    var useBulkRead = false

    override fun init(hardwareMap: HardwareMap) {
        val dcMotors = hardwareMap.dcMotor
        left = dcMotors["intake_right"]
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
        val deltaLeft = -ticksToCM(leftPos - previousLeftPosition)

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

    override fun update() {
        val leftPos: Double
        val rightPos: Double
        val backPos: Double

        if (useBulkRead) {
            leftPos = -TeamRobot.getBulkData2().getMotorCurrentPosition(left).toDouble()
            rightPos = TeamRobot.getBulkData1().getMotorCurrentPosition(right).toDouble()
            backPos = TeamRobot.getBulkData1().getMotorCurrentPosition(back).toDouble()
        } else {
            leftPos = -left.currentPosition.toDouble() // Must return a positive value when moving forward
            rightPos = right.currentPosition.toDouble()
            backPos = back.currentPosition.toDouble()
        }

//        linearUpdate(leftPos, rightPos, backPos)
        updateUsingArcs(leftPos, rightPos, backPos)
    }

    companion object {
        private const val DIAMETER = 7.2
        private const val PULSES_PER_REVOLUTION = 4096
        private const val TICKS_PER_REVOLUTION = 4 * PULSES_PER_REVOLUTION

        private const val DISTANCE_BETWEEN_ENCODER_WHEELS = 19.6125
        private const val DISTANCE_BETWEEN_BACK_ENCODER_AND_CENTER = 14.2 // The distance to the tracking center

        fun ticksToCM(x: Double) = (DIAMETER * Math.PI * x) / TICKS_PER_REVOLUTION
    }
}
