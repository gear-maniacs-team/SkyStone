package net.gearmaniacs.teamcode.hardware.motors

import com.qualcomm.robotcore.hardware.DcMotor.RunMode
import com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.HardwareMap
import net.gearmaniacs.teamcode.utils.IHardware
import net.gearmaniacs.teamcode.utils.getDevice

class Wheels : IHardware {

    lateinit var leftFront: DcMotorEx
        private set
    lateinit var leftBack: DcMotorEx
        private set
    lateinit var rightBack: DcMotorEx
        private set
    lateinit var rightFront: DcMotorEx
        private set

    override fun init(hardwareMap: HardwareMap) {
        leftFront = hardwareMap.getDevice("TL")
        leftBack = hardwareMap.getDevice("BL")
        rightBack = hardwareMap.getDevice("BR")
        rightFront = hardwareMap.getDevice("TR")

        leftFront.direction = DcMotorSimple.Direction.REVERSE
        leftBack.direction = DcMotorSimple.Direction.REVERSE
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
        rightBack.zeroPowerBehavior = behavior
        rightFront.zeroPowerBehavior = behavior
    }

    fun setPowerAll(power: Double) {
        leftFront.power = power
        leftBack.power = power
        rightBack.power = power
        rightFront.power = power
    }

    fun setVelocityAll(velocity: Double) {
        leftFront.velocity = velocity
        leftBack.velocity = velocity
        rightBack.velocity = velocity
        rightFront.velocity = velocity
    }
}
