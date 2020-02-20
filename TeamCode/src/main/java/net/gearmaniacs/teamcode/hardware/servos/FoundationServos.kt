package net.gearmaniacs.teamcode.hardware.servos

import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.Servo
import net.gearmaniacs.teamcode.utils.IHardware
import net.gearmaniacs.teamcode.utils.extensions.getDevice

class FoundationServos : IHardware {

    lateinit var left: Servo
        private set
    lateinit var right: Servo
        private set

    override fun init(hardwareMap: HardwareMap) {
        left = hardwareMap.getDevice("foundation_left")
        right = hardwareMap.getDevice("foundation_right")
    }

    fun attach() {
        left.position = 0.0
        right.position = 0.8
    }

    fun prepareAttach() {
        left.position = 0.5
        right.position = 0.4
    }

    fun detach() {
        left.position = 1.0
        right.position = 0.0
    }
}
