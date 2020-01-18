package org.firstinspires.ftc.teamcode.utils

import com.qualcomm.robotcore.hardware.HardwareMap

inline fun <reified T> HardwareMap.getDevice(deviceName: String): T =
    get(T::class.java, deviceName)

inline fun justTry(block: () -> Unit) {
    try {
        block()
    } catch (e: Exception) {
        e.printStackTrace()
    }
}
