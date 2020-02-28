package org.firstinspires.ftc.teamcode.terminator

import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.HardwareMap

class Wheels {

    lateinit var frontLeft: DcMotorEx
    lateinit var frontRight: DcMotorEx
    lateinit var backLeft: DcMotorEx
    lateinit var backRight: DcMotorEx

    fun init(hardwareMap: HardwareMap) {
        val dcMotors = hardwareMap.dcMotor
        frontLeft = dcMotors["left_front"] as DcMotorEx
        frontRight = dcMotors["right_front"] as DcMotorEx
        backLeft = dcMotors["left_rear"] as DcMotorEx
        backRight = dcMotors["right_rear"] as DcMotorEx

        frontLeft.direction = DcMotorSimple.Direction.REVERSE
        backLeft.direction = DcMotorSimple.Direction.REVERSE
    }

    fun waitForMotors() {
        while (frontLeft.isBusy || frontRight.isBusy || backLeft.isBusy || backRight.isBusy)
            Thread.sleep(10)
    }

    fun setPower(power: Double) {
        frontLeft.power = power
        frontRight.power = power
        backLeft.power = power
        backRight.power = power
    }

    fun setTargetPosition(position: Int) {
        frontLeft.targetPosition = position
        frontRight.targetPosition = position
        backLeft.targetPosition = position
        backRight.targetPosition = position
    }

    fun setMode(runMode: DcMotor.RunMode) {
        frontLeft.mode = runMode
        frontRight.mode = runMode
        backLeft.mode = runMode
        backRight.mode = runMode
    }

    /*fun rotateTicks(ticks : Int, power : Double, direction: DcMotorSimple.Direction) {

       frontLeft.direction = direction
       frontLeft.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
       frontLeft.targetPosition = ticks
       frontLeft.power = power
       frontLeft.mode = DcMotor.RunMode.RUN_TO_POSITION
       waitForMotor()
   }*/

    fun goForwardCm(distance: Double, power: Double) {
        val ticks: Int = cmToTick(distance).toInt()

        frontLeft.startMotor(power, ticks)
        frontRight.startMotor(power, ticks)
        backLeft.startMotor(power, ticks)
        backRight.startMotor(power, ticks)
    }

    fun goBackCm(distance: Double, power: Double) = goForwardCm(distance, -power)

    fun goLeftCm(distance: Double, power: Double) = goRightCm(distance, -power)

    fun goRightCm(distance: Double, power: Double) {
        val ticks: Int = cmToTick(distance).toInt()

        frontLeft.startMotor(power, ticks)
        frontRight.startMotor(-power, ticks)
        backLeft.startMotor(-power, ticks)
        backRight.startMotor(power, ticks)
    }

    companion object {
        const val DIAMETER = 10.0
        const val ENCODER_COUNTS = 537.6

        fun ticToRad(encoderCounts: Double, tickCount: Double): Double = (tickCount * Math.PI * 2) / encoderCounts

        fun radToTick(encoderCounts: Double, rad: Double): Double = (rad * encoderCounts) / (Math.PI * 2)

        fun radToDeg(rad: Double): Double = Math.toDegrees(rad)

        fun degToRad(deg: Double): Double = Math.toRadians(deg)

        fun cmToTick(x: Double): Double = 7.0 // TODO: Finish this function

        fun rpmToTps(rpm: Double, encoderCounts: Double): Double = rpm * (encoderCounts / 60.0)

        private fun DcMotorEx.startMotor(power: Double, target: Int) {
            mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
            targetPosition = target
            this.power = power
            mode = DcMotor.RunMode.RUN_TO_POSITION
        }
    }
}
