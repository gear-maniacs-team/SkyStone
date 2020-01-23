/*
package org.firstinspires.ftc.teamcode.autonomous

import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.util.RobotLog
import org.firstinspires.ftc.robotcore.external.ClassFactory
import org.firstinspires.ftc.robotcore.external.function.Continuation
import org.firstinspires.ftc.robotcore.external.hardware.camera.Camera
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraCaptureSession
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraCaptureSession.StatusCallback
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraException
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName
import org.firstinspires.ftc.robotcore.internal.camera.CameraManagerInternal
import org.firstinspires.ftc.robotcore.internal.system.Deadline
import org.firstinspires.ftc.robotcore.internal.vuforia.externalprovider.CameraMode
import org.firstinspires.ftc.robotcore.internal.vuforia.externalprovider.FrameFormat
import org.firstinspires.ftc.teamcode.detector.TFLiteClassifier
import org.firstinspires.ftc.teamcode.utils.fastLazy
import java.util.concurrent.CountDownLatch
import java.util.concurrent.Executor
import java.util.concurrent.TimeUnit

@TeleOp(name = "Tensorflow")
class TensorflowTest : OpMode() {

    companion object {
        const val WIDTH = 1920
        const val HEIGHT = 1080
    }

    private val classifier by fastLazy { TFLiteClassifier(hardwareMap.appContext) }
    private lateinit var cameraManager: CameraManagerInternal
    private lateinit var cameraName: WebcamName
    private lateinit var camera: Camera
    private lateinit var serialThreadPool: Executor
    private lateinit var cameraCaptureSession: CameraCaptureSession

    override fun init() {
        cameraManager = ClassFactory.getInstance().cameraManager as CameraManagerInternal
        this.serialThreadPool = cameraManager.serialThreadPool
        val cameraMonitorViewId = hardwareMap.appContext.resources.getIdentifier(
            "cameraMonitorViewId",
            "id",
            hardwareMap.appContext.packageName
        )
        cameraName = hardwareMap.get(WebcamName::class.java, "Expensive Webcam")
        camera = cameraManager.requestPermissionAndOpenCamera(Deadline(10, TimeUnit.SECONDS), cameraName, null)

        val captureCallback = CameraCaptureSession.CaptureCallback { session, request, cameraFrame -> }
        val captureStartResult = CountDownLatch(1)


        // Start the Camera
        camera.createCaptureSession(
            Continuation.create(
                serialThreadPool,
                object : CameraCaptureSession.StateCallback {
                    override fun onConfigured(session: CameraCaptureSession) {
                        try {
                            val streamingMode = CameraMode(
                                WIDTH,
                                HEIGHT,
                                30,
                                FrameFormat.YUYV
                            )
                            //Indicate how we want to stream
                            val cameraCaptureRequest = camera.createCaptureRequest(
                                streamingMode.androidFormat,
                                streamingMode.size,
                                streamingMode.framesPerSecond
                            )
                            // Start streaming!
                            session.startCapture(
                                cameraCaptureRequest, captureCallback, Continuation.create(serialThreadPool,
                                    StatusCallback { session, cameraCaptureSequenceId, lastFrameNumber ->
                                        RobotLog.d(
                                            "capture sequence %s reports completed: lastFrame=%d",
                                            cameraCaptureSequenceId,
                                            lastFrameNumber
                                        )
                                    }
                                )
                            )
                        } catch (e: CameraException) {
                            e.printStackTrace()
                            RobotLog.e("exception setting repeat capture request: closing session: %s", session)
                            session.close()
                        } catch (e: RuntimeException) {
                            e.printStackTrace()
                            RobotLog.e("exception setting repeat capture request: closing session: %s", session)
                            session.close()
                        }
                        println("OpenCvWebcam: onConfigured")
                        cameraCaptureSession = session
                        captureStartResult.countDown()
                    }

                    override fun onClosed(session: CameraCaptureSession) {}
                })
        )

        captureStartResult.await(1, TimeUnit.SECONDS)
        telemetry.addData("OpenCvWebcam", "Streaming started")
        telemetry.update()
    }

    override fun loop() {

    }
}

*/