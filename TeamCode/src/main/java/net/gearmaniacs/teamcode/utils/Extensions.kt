package net.gearmaniacs.teamcode.utils

import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.HardwareMap
import org.openftc.revextensions2.RevBulkData

inline fun <reified T> HardwareMap.getDevice(deviceName: String): T =
    get(T::class.java, deviceName)

fun DcMotorEx.getCurrentPosition(bulkInput: RevBulkData?): Int =
    bulkInput?.getMotorCurrentPosition(this) ?: currentPosition

inline fun justTry(block: () -> Unit) {
    try {
        block()
    } catch (e: Exception) {
        e.printStackTrace()
    }
}
