package net.gearmaniacs.teamcode

import android.os.Environment
import com.qualcomm.robotcore.util.RobotLog
import java.io.File

object RobotPos {
    private const val TAG = "RobotPos"
    private const val LAST_POS_FILE = "last_saved_pos"

    @Volatile
    var currentX = 0.0

    @Volatile
    var currentY = 0.0

    @Volatile
    var currentAngle = 0.0 // Stored in Radians

    @Volatile
    var targetX = 0.0

    @Volatile
    var targetY = 0.0

    @Volatile
    var targetAngle = 0.0 // Stored in Radians

    fun resetAll() {
        currentX = 0.0
        currentY = 0.0
        currentAngle = 0.0
        targetX = 0.0
        targetY = 0.0
        targetAngle = 0.0
    }

    fun deleteLastSavedPosition() {
        val file =
            File(Environment.getExternalStorageDirectory(), LAST_POS_FILE)
        file.delete()
    }

    fun saveCurrentPosition() {
        val file =
            File(Environment.getExternalStorageDirectory(), LAST_POS_FILE)
        val text = "$currentX\n$currentY\n$currentAngle\n"
        file.writeText(text)
    }

    fun loadLastPosition() {
        val file =
            File(Environment.getExternalStorageDirectory(), LAST_POS_FILE)
        if (!file.exists()) {
            RobotLog.ee(TAG, "No last position found")
            return
        }
        val lines = file.readLines()
        try {
            currentX = lines[0].toDouble()
            currentY = lines[1].toDouble()
            currentAngle = lines[2].toDouble()
        } catch (e: Exception) {
            RobotLog.ee(TAG, e, "Failed to parse last saved position")
        }
    }
}
