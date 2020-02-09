package net.gearmaniacs.teamcode;

import android.os.Environment;

import com.qualcomm.robotcore.util.RobotLog;

import java.io.File;
import java.util.List;

import kotlin.io.FilesKt;
import kotlin.text.Charsets;

public final class RobotPos {

    private RobotPos() {
    }

    private final static String TAG = "RobotPos";
    private final static String LAST_POS_FILE = "last_saved_pos";

    public static volatile double currentX = 0.0;
    public static volatile double currentY = 0.0;
    public static volatile double currentAngle = 0.0; // Stored in Radians

    public static volatile double targetX = 0.0;
    public static volatile double targetY = 0.0;
    public static volatile double targetAngle = 0.0; // Stored in Radians

    public static void resetAll() {
        currentX = 0.0;
        currentY = 0.0;
        currentAngle = 0.0;

        targetX = 0.0;
        targetY = 0.0;
        targetAngle = 0.0;
    }

    public static void deleteLastSavedPosition() {
        File file = new File(Environment.getExternalStorageDirectory(), LAST_POS_FILE);
        file.delete();
    }

    public static void saveCurrentPosition() {
        File file = new File(Environment.getExternalStorageDirectory(), LAST_POS_FILE);

        String text = String.valueOf(currentX) +
                '\n' +
                currentY +
                '\n' +
                currentAngle;

        FilesKt.writeText(file, text, Charsets.UTF_8);
    }

    public static void loadLastPosition() {
        File file = new File(Environment.getExternalStorageDirectory(), LAST_POS_FILE);
        if (!file.exists()) {
            RobotLog.ee(TAG, "No last position found");
            return;
        }

        List<String> lines = FilesKt.readLines(file, Charsets.UTF_8);

        try {
            currentX = Double.parseDouble(lines.get(0));
            currentY = Double.parseDouble(lines.get(1));
            currentAngle = Double.parseDouble(lines.get(2));
        } catch (Exception e) {
            RobotLog.ee(TAG, e, "Failed to parse last saved position");
        }
    }
}
