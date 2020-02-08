package net.gearmaniacs.teamcode.detector

import android.content.Context
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.util.RobotLog
import net.gearmaniacs.teamcode.utils.getDevice
import org.firstinspires.ftc.robotcore.external.ClassFactory
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer
import org.firstinspires.ftc.robotcore.external.tfod.Recognition
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector
import kotlin.concurrent.thread

class VuforiaManager {

    private val lock = Any()
    @Volatile
    private var initializing = false
    private var tfod: TFObjectDetector? = null

    fun startDetectorAsync(hardwareMap: HardwareMap) {
        check(ClassFactory.getInstance().canCreateTFObjectDetector()) { "This device is not compatible with TFOD!" }

        if (initializing) return

        val webcamName = hardwareMap.getDevice<WebcamName>(DEFAULT_CAMERA_NAME)

        thread(start = true, name = "Vuforia Initializer") {
            try {
                synchronized(lock) {
                    initializing = true

                    val localizer = getLocalizerInstance(webcamName)
                    initTfod(hardwareMap.appContext, localizer)

                    initializing = false
                }
            } catch (e: Exception) {
                RobotLog.ee(TAG, e, "Initialization Failed")
            }
        }
    }

    val recognitions: List<Recognition>
        get() = tfod?.recognitions ?: emptyList()

    fun activateDetector() {
        try {
            synchronized(lock) {
                tfod!!.activate()
            }
        } catch (e: Exception) {
            RobotLog.ee("VuforiaManager", e, "Detector activation failed")
        }
    }

    fun deactivateDetector() {
        synchronized(lock) {
            tfod?.deactivate()
        }
    }

    fun stopCamera() {
        synchronized(lock) {
            tfod?.let {
                it.deactivate()
                it.shutdown()
            }
            tfod = null
        }
    }

    private fun getLocalizerInstance(webcamName: WebcamName): VuforiaLocalizer {
        val parameters = VuforiaLocalizer.Parameters().apply {
            vuforiaLicenseKey = VUFORIA_KEY
            cameraName = webcamName
        }

        return ClassFactory.getInstance().createVuforia(parameters)
    }

    private fun initTfod(context: Context, vuforiaLocalizer: VuforiaLocalizer) {
        val tfodMonitorViewId =
            context.resources.getIdentifier("tfodMonitorViewId", "id", context.packageName)

        val tfodParams = TFObjectDetector.Parameters(tfodMonitorViewId)
        tfodParams.minimumConfidence = 0.1

        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParams, vuforiaLocalizer).apply {
            loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_STONE, LABEL_SKYSTONE)
        }
    }

    companion object {
        private const val TAG = "VuforiaManager"

        private const val DEFAULT_CAMERA_NAME = "Expensive Webcam"
        private const val TFOD_MODEL_ASSET = "CustomSkystone.tflite"
        const val LABEL_STONE = "stone"
        const val LABEL_SKYSTONE = "skystone"

        private const val VUFORIA_KEY =
            "AZnVnoj/////AAABmdXzVSC7bkZik9EURkca9g5GwHTQjL0SB5CABkSEajM1oX/nSOWoXxcxH/watnjKf3WlWcGhyPvV0E8eMNZmTbTgrB/8OJhqAflMV+CjgBtERmweuXjLiPcvEgJNrZD7USn+LK53L0VuSYdi4NwJxy7ypbse7jbXlOmJVgogCXbD4+yjYDbnVmBkkMQMhLgIFQZ0wRApvdxc7R/O/rhsQfWrWWekxjIp4wNeYh5JBsCrCRjdPu1P7QLKAMSOpK5lXqJjmD36TPDxqrQEGfdKxkMe2SJta/3tyzc+v/mFRmNDJjqVMYu69eEy6jh7u/KQA2Uj4pdcIfnZhMWwBO58guP2TPl5HCof4weEEUI6ZF8w"
    }
}
