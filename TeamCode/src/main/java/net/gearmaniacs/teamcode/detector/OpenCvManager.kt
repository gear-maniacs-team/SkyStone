package net.gearmaniacs.teamcode.detector

import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName
import org.openftc.easyopencv.OpenCvCamera
import org.openftc.easyopencv.OpenCvCameraRotation
import org.openftc.easyopencv.OpenCvPipeline
import org.openftc.easyopencv.OpenCvWebcam

class OpenCvManager(private var pipeline: OpenCvPipeline) {

    private lateinit var camera: OpenCvWebcam

    fun startCamera(hardwareMap: HardwareMap) {
        val cameraMonitorViewId = hardwareMap.appContext.resources.getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.packageName)
        camera = OpenCvWebcam(hardwareMap.get(WebcamName::class.java, "Expensive Webcam"), cameraMonitorViewId)

        camera.openCameraDevice()
        camera.setPipeline(pipeline)
    }

    fun startDetector(width: Int, height: Int) {
        camera.startStreaming(width, height, OpenCvCameraRotation.UPRIGHT)
    }

    fun stopDetector() {
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
    }

    fun stopCamera() {
        camera.stopStreaming()
        camera.closeCameraDevice()
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
