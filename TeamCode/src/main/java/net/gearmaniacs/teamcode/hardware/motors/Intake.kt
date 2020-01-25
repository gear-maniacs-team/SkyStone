package net.gearmaniacs.teamcode.hardware.motors

import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.HardwareMap
import net.gearmaniacs.teamcode.utils.IHardware

class Intake : IHardware {

    lateinit var left: DcMotorEx
        private set
    lateinit var right: DcMotorEx
        private set

    override fun init(hardwareMap: HardwareMap) {
        val dcMotors = hardwareMap.dcMotor
        left = dcMotors["intake_left"] as DcMotorEx
        right = dcMotors["intake_right"] as DcMotorEx

        left.direction = DcMotorSimple.Direction.FORWARD
        right.direction = DcMotorSimple.Direction.REVERSE
    }

    fun setPowerAll(power: Double) {
        left.power = power
        right.power = power
    }
}
