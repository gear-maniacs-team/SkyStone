package org.firstinspires.ftc.teamcode.sensors

import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotor.RunMode
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.robotcore.internal.system.AppUtil
import org.firstinspires.ftc.teamcode.RobotPos
import org.firstinspires.ftc.teamcode.utils.IHardware
import org.firstinspires.ftc.teamcode.utils.IUpdatable
import kotlin.math.cos
import kotlin.math.sin

class Encoder : IHardware, IUpdatable {

    lateinit var left: DcMotor
        private set
    lateinit var right: DcMotor
        private set
    lateinit var back: DcMotor
        private set

    private var changeInAngle = 0.0
    private var previousLeftPosition = 0.0
    private var previousRightPosition = 0.0
    private var previousBackPosition = 0.0

    private var robotEncoderWheelDistance = 0.0
    private var horizontalEncoderTickPerDegreeOffset = 0.0
    private var robotBackEncoderWheelDistance = 0.0 // The distance to the tracking center

    override fun init(hardwareMap: HardwareMap) {
        val dcMotors = hardwareMap.dcMotor
        left = dcMotors["TL"]
        right = dcMotors["BL"]
        back = dcMotors["TR"]

        robotEncoderWheelDistance = AppUtil.getInstance()
            .getSettingsFile("wheelBaseSeparation.txt")
            .readText()
            .trim()
            .toDouble()// * COUNTS_PER_INCH

        horizontalEncoderTickPerDegreeOffset = AppUtil.getInstance()
            .getSettingsFile("horizontalTickOffset.txt")
            .readText()
            .trim()
            .toDouble()

        // Both lateral encoders should return a positive value when moving forwards
        left.direction = DcMotorSimple.Direction.REVERSE
        // The back encoder should return a positive value when moving to the right
        back.direction = DcMotorSimple.Direction.REVERSE

        setModeAll(RunMode.STOP_AND_RESET_ENCODER)
        setModeAll(RunMode.RUN_USING_ENCODER)
    }

    fun setModeAll(mode: RunMode) {
        left.mode = mode
        right.mode = mode
        back.mode = mode
    }

    fun updateUsingLines() {
        val leftPosition = left.currentPosition.toDouble()
        val rightPosition = right.currentPosition.toDouble()
        val backPosition = back.currentPosition.toDouble()

        val leftChange = leftPosition - previousLeftPosition
        val rightChange = rightPosition - previousRightPosition

        // Calculate Angle
        changeInAngle = (leftChange - rightChange) / robotEncoderWheelDistance
        val newCurrentAngle = RobotPos.currentAngle + changeInAngle

        // Get the components of the motion
        val rawHorizontalChange = backPosition - previousBackPosition
        val horizontalChange = rawHorizontalChange - changeInAngle * horizontalEncoderTickPerDegreeOffset

        val averageChange = (leftChange + rightChange) / 2

        // Calculate and update the position values
        RobotPos.currentX += averageChange * sin(newCurrentAngle) + horizontalChange * cos(newCurrentAngle)
        RobotPos.currentY += averageChange * cos(newCurrentAngle) - horizontalChange * sin(newCurrentAngle)
        RobotPos.currentAngle = newCurrentAngle

        previousRightPosition = leftPosition
        previousLeftPosition = rightPosition
        previousBackPosition = backPosition
    }

    fun updateUsingArcs() {
        val leftPosition = left.currentPosition.toDouble()
        val rightPosition = right.currentPosition.toDouble()
        val backPosition = back.currentPosition.toDouble()

        val leftChange = leftPosition - previousLeftPosition
        val rightChange = rightPosition - previousRightPosition
        val backChange = backPosition - previousBackPosition

        // Calculate Angle
        changeInAngle = (leftChange - rightChange) / robotEncoderWheelDistance

        // Get the components of the motion
        val newX = 2 * sin(RobotPos.currentAngle / 2) * (backChange / changeInAngle + robotBackEncoderWheelDistance)
        val newY = 2 * sin(RobotPos.currentAngle / 2) * (rightChange / changeInAngle + robotEncoderWheelDistance / 2)
        val averageOrientation = RobotPos.currentAngle + changeInAngle / 2

        // Calculate and update the position values
        // Rotate the cartesian coordinate system by transforming into polar form, adding the angle and then
        // transforming back into cartesian form.
        RobotPos.currentX += newX * cos(averageOrientation) + newY * sin(averageOrientation)
        RobotPos.currentY += -newX * sin(averageOrientation) + newY * cos(averageOrientation)
        RobotPos.currentAngle += changeInAngle

        previousRightPosition = leftPosition
        previousLeftPosition = rightPosition
        previousBackPosition = backPosition
    }

    override fun update() {
        updateUsingLines()
    }
}
