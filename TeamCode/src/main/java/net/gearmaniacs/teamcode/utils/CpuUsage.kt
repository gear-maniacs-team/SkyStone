package net.gearmaniacs.teamcode.utils

import java.io.DataInputStream
import java.io.File
import java.io.FileFilter
import java.util.regex.Pattern

object CpuUsage {

    private val cpuCoresNumber by lazy {
        val cpuFilter = object : FileFilter {
            override fun accept(pathname: File?): Boolean {
                pathname ?: return false
                // Check if filename is "cpu", followed by a single digit number
                return Pattern.matches("cpu[0-9]+", pathname.name)
            }
        }

        try {
            val cpuInfoFolder = File("/sys/devices/system/cpu/")
            cpuInfoFolder.listFiles(cpuFilter)!!.size
        } catch (e: Exception) {
            Runtime.getRuntime().availableProcessors()
        }
    }

    private fun readSystemFile(systemFile: String): Int {
        val process = ProcessBuilder("/system/bin/cat", systemFile).start()

        DataInputStream(process.inputStream).use {
            return it.readInt()
        }
    }

    private fun getCurrentCpuFrequency() = IntArray(cpuCoresNumber) { index ->
        val path = "/sys/devices/system/cpu/cpu$index/cpufreq/scaling_cur_freq"
        readSystemFile(path)
    }

    private fun getMaxCpuFrequency() = IntArray(cpuCoresNumber) { index ->
        val path = "/sys/devices/system/cpu/cpu$index/cpufreq/cpuinfo_max_freq"
        readSystemFile(path)
    }

    val usage: FloatArray
        get() {
            val current = getCurrentCpuFrequency()
            val max = getMaxCpuFrequency()

            return FloatArray(cpuCoresNumber) { index ->
                current[index].toFloat() / max[index]
            }
        }

    val totalUsage: Float
        get() = usage.average().toFloat()
}
