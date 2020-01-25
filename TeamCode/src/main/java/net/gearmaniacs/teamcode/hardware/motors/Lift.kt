package net.gearmaniacs.teamcode.hardware.motors

import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.HardwareMap
import net.gearmaniacs.teamcode.utils.IHardware

class Lift : IHardware {
    
    lateinit var left: DcMotorEx
        private set
    lateinit var right: DcMotorEx
        private set
    
    override fun init(hardwareMap: HardwareMap) {
        val dcMotors = hardwareMap.dcMotor
        left = dcMotors["lift_left"] as DcMotorEx
        right = dcMotors["lift_right"] as DcMotorEx

        with(left) {
            mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
            targetPosition = 0
            mode = DcMotor.RunMode.RUN_TO_POSITION
        }

        with(right) {
            mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
            targetPosition = 0
            mode = DcMotor.RunMode.RUN_TO_POSITION
        }
    }
}
