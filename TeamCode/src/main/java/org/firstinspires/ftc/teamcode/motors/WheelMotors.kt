package org.firstinspires.ftc.teamcode.motors

import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.HardwareMap

class WheelMotors(dcMotors: HardwareMap.DeviceMapping<DcMotor>) {

    val leftFront: DcMotorEx = dcMotors["TL"] as DcMotorEx
    val leftBack: DcMotorEx = dcMotors["BL"] as DcMotorEx
    val rightFront: DcMotorEx = dcMotors["TR"] as DcMotorEx
    val rightBack: DcMotorEx = dcMotors["BR"] as DcMotorEx

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
