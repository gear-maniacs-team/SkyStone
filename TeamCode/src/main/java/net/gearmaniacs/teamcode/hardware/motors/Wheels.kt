package net.gearmaniacs.teamcode.hardware.motors

import com.qualcomm.robotcore.hardware.DcMotor.RunMode
import com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.HardwareMap
import net.gearmaniacs.teamcode.utils.IHardware

class Wheels : IHardware {

    lateinit var leftFront: DcMotorEx
        private set
    lateinit var leftBack: DcMotorEx
        private set
    lateinit var rightFront: DcMotorEx
        private set
    lateinit var rightBack: DcMotorEx
        private set
    
    override fun init(hardwareMap: HardwareMap) {
        val dcMotors = hardwareMap.dcMotor
        leftFront = dcMotors["TL"] as DcMotorEx
        leftBack = dcMotors["BL"] as DcMotorEx
        rightFront = dcMotors["TR"] as DcMotorEx
        rightBack = dcMotors["BR"] as DcMotorEx
    }

    fun setModeAll(mode: RunMode) {
        leftFront.mode = mode
        leftBack.mode = mode
        rightFront.mode = mode
        rightBack.mode = mode
    }

    fun setZeroPowerBehaviorAll(behavior: ZeroPowerBehavior) {
        leftFront.zeroPowerBehavior = behavior
        leftBack.zeroPowerBehavior = behavior
        rightFront.zeroPowerBehavior = behavior
        rightBack.zeroPowerBehavior = behavior
    }

    fun setPowerAll(power: Double) {
        leftFront.power = power
        leftBack.power = power
        rightFront.power = power
        rightBack.power = power
    }
}
