package org.firstinspires.ftc.teamcode.motors

import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.HardwareMap

class SimpleMotors(deviceMapping: HardwareMap.DeviceMapping<DcMotor>) {

    val left: DcMotor = deviceMapping["left"]
    val right: DcMotor = deviceMapping["right"]

    fun setModeAll(mode: DcMotor.RunMode) {
        left.mode = mode
        right.mode = mode
    }

    fun setTargetPositionAll(position: Int) {
        left.targetPosition = position
        right.targetPosition = position
    }

    fun setPowerAll(power: Double) {
        left.power = power
        right.power = power
    }
}
