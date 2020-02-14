package net.gearmaniacs.teamcode.hardware.servos

import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.Servo
import net.gearmaniacs.teamcode.utils.IHardware
import net.gearmaniacs.teamcode.utils.extensions.getDevice

class OuttakeServos : IHardware {

    lateinit var left: Servo
        private set
    lateinit var right: Servo
        private set

    override fun init(hardwareMap: HardwareMap) {
        left = hardwareMap.getDevice("out_left")
        right = hardwareMap.getDevice("out_right")
    }

    fun extend() {
        left.position = 0.0
        right.position = 1.0
    }

    fun semiExtend() {
        left.position = 0.5
        right.position = 0.5
    }

    fun retract() {
        left.position = 0.8
        right.position = 0.2
    }
}
