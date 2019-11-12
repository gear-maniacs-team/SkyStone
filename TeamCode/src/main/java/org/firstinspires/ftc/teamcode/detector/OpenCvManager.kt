package org.firstinspires.ftc.teamcode.detector

import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName
import org.openftc.easyopencv.OpenCvCamera
import org.openftc.easyopencv.OpenCvCameraRotation
import org.openftc.easyopencv.OpenCvPipeline
import org.openftc.easyopencv.OpenCvWebcam

class OpenCvManager(private var pipeline: OpenCvPipeline) {

    private lateinit var webcam: OpenCvCamera

    fun startCamera(hardwareMap: HardwareMap) {
        val cameraMonitorViewId = hardwareMap.appContext.resources.getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.packageName)
        webcam = OpenCvWebcam(hardwareMap.get(WebcamName::class.java, "Webcam 1"), cameraMonitorViewId)

        webcam.openCameraDevice()
        webcam.setPipeline(pipeline)
    }

    fun startDetector(width: Int, height: Int) {
        webcam.startStreaming(width, height, OpenCvCameraRotation.UPRIGHT)
    }

    fun stopDetector() {
        webcam.stopStreaming()
    }

    fun showPreview(showFps: Boolean = false) {
        webcam.resumeViewport()
        webcam.showFpsMeterOnViewport(showFps)
    }

    fun hidePreview() {
        webcam.pauseViewport()
    }

    fun switchPipeline(pipeline: OpenCvPipeline) {
        this.pipeline = pipeline
    }

    fun stopCamera() {
        webcam.stopStreaming()
        webcam.closeCameraDevice()
    }

    fun printTelemetry(telemetry: Telemetry) {
        telemetry.addData("Frame Count", webcam.frameCount)
        telemetry.addData("FPS", String.format("%.2f", webcam.fps))
        telemetry.addData("Total frame time ms", webcam.totalFrameTimeMs)
        telemetry.addData("Pipeline time ms", webcam.pipelineTimeMs)
        telemetry.addData("Overhead time ms", webcam.overheadTimeMs)
    }
}
