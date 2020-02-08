package net.gearmaniacs.teamcode.hardware.motors

import com.qualcomm.robotcore.hardware.DcMotor.RunMode
import com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.HardwareMap
import net.gearmaniacs.teamcode.utils.IHardware
import net.gearmaniacs.teamcode.utils.getDevice

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
        leftFront = hardwareMap.getDevice("TL")
        leftBack = hardwareMap.getDevice("BL")
        rightFront = hardwareMap.getDevice("TR")
        rightBack = hardwareMap.getDevice("BR")
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
        rightFront.power = -power
        rightBack.power = -power
        //TODO: Fix
    }

    fun setVelocityAll(velocity: Double) {
        leftFront.velocity = velocity
        leftBack.velocity = velocity
        rightFront.velocity = -velocity
        rightBack.velocity = -velocity
    }
}
