package net.gearmaniacs.teamcode.hardware.motors

import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.HardwareMap
import net.gearmaniacs.teamcode.utils.IHardware
import net.gearmaniacs.teamcode.utils.getDevice

class Lift : IHardware {
    
    lateinit var left: DcMotorEx
        private set
    lateinit var right: DcMotorEx
        private set
    
    override fun init(hardwareMap: HardwareMap) {
        left = hardwareMap.getDevice("lift_left")
        right = hardwareMap.getDevice("lift_right")

        with(left) {
            direction = DcMotorSimple.Direction.REVERSE
            mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
            targetPosition = 0
            mode = DcMotor.RunMode.RUN_TO_POSITION
        }

        with(right) {
            direction = DcMotorSimple.Direction.FORWARD
            mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
            targetPosition = 0
            mode = DcMotor.RunMode.RUN_TO_POSITION
        }
    }

    fun setTargetPositionAll(targetPosition: Int) {
        left.targetPosition = targetPosition
        right.targetPosition = targetPosition
    }

    fun setPowerAll(power: Double) {
        left.power = power
        right.power = power
    }
}
