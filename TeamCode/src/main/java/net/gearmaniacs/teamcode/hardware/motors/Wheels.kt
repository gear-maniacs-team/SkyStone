package net.gearmaniacs.teamcode.hardware.motors

import com.qualcomm.robotcore.hardware.DcMotor.RunMode
import com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.HardwareMap
import net.gearmaniacs.teamcode.utils.IHardware
import net.gearmaniacs.teamcode.utils.extensions.getDevice

class Wheels : IHardware {

    private var all = emptyList<DcMotorEx>()
    lateinit var leftFront: DcMotorEx
        private set
    lateinit var leftRear: DcMotorEx
        private set
    lateinit var rightRear: DcMotorEx
        private set
    lateinit var rightFront: DcMotorEx
        private set

    override fun init(hardwareMap: HardwareMap) {
        leftFront = hardwareMap.getDevice("left_front")
        leftRear = hardwareMap.getDevice("left_rear")
        rightRear = hardwareMap.getDevice("right_rear")
        rightFront = hardwareMap.getDevice("right_front")

        leftFront.direction = DcMotorSimple.Direction.REVERSE
        leftRear.direction = DcMotorSimple.Direction.REVERSE

        all = mutableListOf(leftFront, leftRear, rightFront, leftRear)
    }

    fun setModeAll(mode: RunMode) {
        all.forEach { it.mode = mode }
    }

    fun setZeroPowerBehaviorAll(behavior: ZeroPowerBehavior) {
        all.forEach { it.zeroPowerBehavior = behavior }
    }

    fun setPowerAll(power: Double) {
        all.forEach { it.power = power }
    }

    fun setVelocityAll(velocity: Double) {
        all.forEach { it.velocity = velocity }
    }
}
