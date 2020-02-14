package net.gearmaniacs.teamcode.hardware.motors

import com.qualcomm.robotcore.hardware.DcMotor.RunMode
import com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.HardwareMap
import net.gearmaniacs.teamcode.utils.IHardware
import net.gearmaniacs.teamcode.utils.extensions.getDevice

class Wheels : IHardware {

    private var all: List<DcMotorEx> = emptyList()

    val leftFront: DcMotorEx
        get() = all[0]
    val leftRear: DcMotorEx
        get() = all[1]
    val rightRear: DcMotorEx
        get() = all[2]
    val rightFront: DcMotorEx
        get() = all[3]

    override fun init(hardwareMap: HardwareMap) {
        val leftFront = hardwareMap.getDevice<DcMotorEx>("left_front")
        val leftRear = hardwareMap.getDevice<DcMotorEx>("left_rear")
        val rightRear = hardwareMap.getDevice<DcMotorEx>("right_rear")
        val rightFront = hardwareMap.getDevice<DcMotorEx>("right_front")

        leftFront.direction = DcMotorSimple.Direction.REVERSE
        leftRear.direction = DcMotorSimple.Direction.REVERSE

        // Make sure the order of the motors match their index in the getters
        all = listOf(leftFront, leftRear, rightRear, rightFront)
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
