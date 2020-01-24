package net.gearmaniacs.teamcode.utils

import com.qualcomm.robotcore.hardware.HardwareMap

interface IHardware {
    fun init(hardwareMap: HardwareMap)

    fun start() = Unit

    fun stop() = Unit
}
