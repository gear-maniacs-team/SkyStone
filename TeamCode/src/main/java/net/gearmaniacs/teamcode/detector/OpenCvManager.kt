package net.gearmaniacs.teamcode.detector

import android.util.Log
import com.qualcomm.robotcore.hardware.HardwareMap
import net.gearmaniacs.teamcode.utils.IHardware
import net.gearmaniacs.teamcode.utils.extensions.getDevice
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName
import org.openftc.easyopencv.*
import java.lang.Exception

class OpenCvManager(private var pipeline: OpenCvPipeline) : IHardware {

    private lateinit var camera: OpenCvCamera

    override fun init(hardwareMap: HardwareMap) {
        val cameraMonitorViewId = hardwareMap.appContext.resources.getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.packageName)
        val webcamName = hardwareMap.getDevice<WebcamName>("Expensive Webcam")
        camera = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId)

        camera.openCameraDevice()
        camera.setPipeline(pipeline)
    }

    override fun start() {
        start(640, 480)
    }

    fun start(width: Int, height: Int) {
        camera.startStreaming(width, height, OpenCvCameraRotation.UPRIGHT)
    }

    fun tryInitAndStart(hardwareMap: HardwareMap): Boolean {
        Log.v("CpenCvManager", "Trying to start Camera")
        return try {
            init(hardwareMap)
            start()
            Log.v("CpenCvManager", "Camera started successfully")
            true
        } catch (e: Exception) {
            e.printStackTrace()
            Log.v("CpenCvManager", "Camera strat failed")
            false
        }
    }

    override fun stop() {
        camera.stopStreaming()
    }

    fun showPreview(showFps: Boolean = false) {
        camera.resumeViewport()
        camera.showFpsMeterOnViewport(showFps)
    }

    fun hidePreview() {
        camera.pauseViewport()
    }

    fun switchPipeline(pipeline: OpenCvPipeline) {
        this.pipeline = pipeline
        camera.setPipeline(pipeline)
    }

    fun getPipeline() = pipeline

    fun printTelemetry(telemetry: Telemetry) {
        telemetry.addData("Frame Count", camera.frameCount)
        telemetry.addData("FPS", String.format("%.2f", camera.fps))
        telemetry.addData("Total frame time ms", camera.totalFrameTimeMs)
        telemetry.addData("Pipeline time ms", camera.pipelineTimeMs)
        telemetry.addData("Overhead time ms", camera.overheadTimeMs)
    }
}

