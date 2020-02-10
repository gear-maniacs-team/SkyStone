package net.gearmaniacs.teamcode.hardware.motors

import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.HardwareMap
import net.gearmaniacs.teamcode.utils.IHardware
import net.gearmaniacs.teamcode.utils.getDevice

class Intake : IHardware {

    lateinit var left: DcMotorEx
        private set
    lateinit var right: DcMotorEx
        private set

    override fun init(hardwareMap: HardwareMap) {
        left = hardwareMap.getDevice("intake_left")
        right = hardwareMap.getDevice("intake_right")

        left.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        right.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
    }

    fun setPowerAll(power: Double) {
        left.power = power
        right.power = -power
    }
}
