package net.gearmaniacs.teamcode.hardware.servos

import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.Servo
import net.gearmaniacs.teamcode.utils.IHardware
import net.gearmaniacs.teamcode.utils.extensions.getDevice

class OuttakeServos : IHardware {

    private var all = emptyList<Servo>()

    val leftExtension: Servo
        get() = all[0]
    val rightExtension: Servo
        get() = all[1]
    val spinner: Servo
        get() = all[2]
    val gripper: Servo
        get() = all[3]

    override fun init(hardwareMap: HardwareMap) {
        val leftExtension = hardwareMap.getDevice<Servo>("out_left")
        val rightExtension = hardwareMap.getDevice<Servo>("out_right")
        val spinner = hardwareMap.getDevice<Servo>("spinner")
        val gripper = hardwareMap.getDevice<Servo>("gripper")

        all = listOf(leftExtension, rightExtension, spinner, gripper)
    }

    fun extend() {
        leftExtension.position = 0.0
        rightExtension.position = 1.0
    }

    fun semiExtend() {
        leftExtension.position = 0.6
        rightExtension.position = 0.4
    }

    fun retract() {
        leftExtension.position = 1.0
        rightExtension.position = 0.0
    }

    fun activateGripper() {
        gripper.position = 0.8
    }

    fun releaseGripper() {
        gripper.position = 0.44
    }

    fun activateSpinner() {
        spinner.position = 0.15
    }

    fun deactivateSpinner() {
        spinner.position = 0.95
    }
}
