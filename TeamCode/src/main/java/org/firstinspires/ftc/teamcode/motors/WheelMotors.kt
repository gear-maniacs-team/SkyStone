package org.firstinspires.ftc.teamcode.motors

import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.HardwareMap

class WheelMotors(dcMotors: HardwareMap.DeviceMapping<DcMotor>) {

    val leftFront: DcMotor = dcMotors["TL"]
    val leftBack: DcMotor = dcMotors["BL"]
    val rightFront: DcMotor = dcMotors["TR"]
    val rightBack: DcMotor = dcMotors["BR"]

    fun setModeAll(mode: DcMotor.RunMode) {
        leftFront.mode = mode
        leftBack.mode = mode
        rightFront.mode = mode
        rightBack.mode = mode
    }

    fun setTargetPositionAll(position: Int) {
        leftFront.targetPosition = position
        leftBack.targetPosition = position
        rightFront.targetPosition = position
        rightBack.targetPosition = position
    }

    fun setPowerAll(power: Double) {
        leftFront.power = power
        leftBack.power = power
        rightFront.power = power
        rightBack.power = power
    }
}
