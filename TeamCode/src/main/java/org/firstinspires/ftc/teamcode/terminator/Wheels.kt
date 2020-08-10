package org.firstinspires.ftc.teamcode.terminator

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.HardwareMap
import net.gearmaniacs.teamcode.RobotPos
import net.gearmaniacs.teamcode.utils.IHardware
import net.gearmaniacs.teamcode.utils.MathUtils

class Wheels : IHardware {

    lateinit var frontLeft: DcMotorEx
    lateinit var frontRight: DcMotorEx
    lateinit var backLeft: DcMotorEx
    lateinit var backRight: DcMotorEx

    override fun init(hardwareMap: HardwareMap) {
        val dcMotors = hardwareMap.dcMotor
        frontLeft = dcMotors["left_front"] as DcMotorEx
        frontRight = dcMotors["right_front"] as DcMotorEx
        backLeft = dcMotors["left_rear"] as DcMotorEx
        backRight = dcMotors["right_rear"] as DcMotorEx

        frontLeft.direction = DcMotorSimple.Direction.FORWARD
        backLeft.direction = DcMotorSimple.Direction.FORWARD
        backRight.direction = DcMotorSimple.Direction.REVERSE
        frontRight.direction = DcMotorSimple.Direction.REVERSE
    }

    fun waitForMotors() {
        while (frontLeft.isBusy && frontRight.isBusy && backLeft.isBusy && backRight.isBusy) {
            val packet = TelemetryPacket().apply {
                put("frontLeft encoder", frontLeft.currentPosition)
                put("frontRight encoder", frontRight.currentPosition)
                put("backLeft encoder", backLeft.currentPosition)
                put("backRight encoder", backRight.currentPosition)
            }
            FtcDashboard.getInstance().sendTelemetryPacket(packet)
            Thread.sleep(10)
        }
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

    fun goForwardTick(ticks: Int, power: Double) {
        frontLeft.startMotor(power, ticks)
        frontRight.startMotor(power, ticks)
        backLeft.startMotor(power, ticks)
        backRight.startMotor(power, ticks)
    }

    fun goForwardCm(distance: Double, power: Double) {
        val ticks: Int = cmToTick(distance).toInt()

        frontLeft.startMotor(power, ticks)
        frontRight.startMotor(power, ticks)
        backLeft.startMotor(power, ticks)
        backRight.startMotor(power, ticks)
    }

    fun goBackCm(distance: Double, power: Double) = goForwardCm(-distance, power)

    fun goLeftCm(distance: Double, power: Double) = goRightCm(-distance, power)

    fun goRightCm(distance: Double, power: Double) {
        val ticks: Int = cmToTick(distance).toInt()

        frontLeft.startMotor(power, ticks)
        frontRight.startMotor(power, -ticks)
        backLeft.startMotor(power, -ticks)
        backRight.startMotor(power, ticks)
    }

    fun rotateLeft(distance: Double, power: Double) = rotateRight(-distance, power)

    fun rotateRight(distance: Double, power: Double) {
        val ticks = cmToTick(distance).toInt()

        frontLeft.startMotor(power, ticks)
        frontRight.startMotor(power, ticks)
        backLeft.startMotor(power, -ticks)
        backRight.startMotor(power, -ticks)
    }

    fun goRightForward(distance: Double, power: Double){
        val ticks = cmToTick(distance).toInt()

        frontLeft.startMotor(power,ticks)
        backRight.startMotor(power, ticks)
    }

    fun goLeftBack(distance: Double,power: Double) = goRightForward(-distance, power)

    fun goLeftForward(distance: Double, power: Double){
        val ticks = cmToTick(distance).toInt()

        frontRight.startMotor(power, ticks)
        backLeft.startMotor(power, ticks)
    }

    fun goRightBack(distance: Double, power: Double) = goLeftForward(-distance, power)

    companion object {
        private const val DIAMETER = 10.0

        private const val ENCODER_COUNTS = 537.6

        fun ticToRad(encoderCounts: Double, tickCount: Double): Double = (tickCount * Math.PI * 2) / encoderCounts

        fun radToTick(encoderCounts: Double, rad: Double): Double = (rad * encoderCounts) / (Math.PI * 2)

        fun radToDeg(rad: Double): Double = Math.toDegrees(rad)

        fun degToRad(deg: Double): Double = Math.toRadians(deg)

        fun cmToTick(cm: Double): Double = (cm * ENCODER_COUNTS) / (DIAMETER * Math.PI)

        fun rpmToTps(rpm: Double, encoderCounts: Double): Double = rpm * (encoderCounts / 60.0)

        private fun DcMotorEx.startMotor(power: Double, target: Int) {
            mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
            targetPosition = target
            mode = DcMotor.RunMode.RUN_TO_POSITION
            this.power = power
        }
    }
}
