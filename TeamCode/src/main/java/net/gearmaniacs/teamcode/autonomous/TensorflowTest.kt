package net.gearmaniacs.teamcode.autonomous

import android.util.Log
import android.view.View
import android.view.ViewGroup
import android.widget.LinearLayout
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.util.RobotLog
import net.gearmaniacs.teamcode.detector.PreviewSurfaceView
import net.gearmaniacs.teamcode.detector.TFLiteClassifier
import net.gearmaniacs.teamcode.utils.fastLazy
import org.firstinspires.ftc.robotcore.external.ClassFactory
import org.firstinspires.ftc.robotcore.external.android.util.Size
import org.firstinspires.ftc.robotcore.external.function.Continuation
import org.firstinspires.ftc.robotcore.external.hardware.camera.Camera
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraCaptureSession
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraCaptureSession.StatusCallback
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraException
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName
import org.firstinspires.ftc.robotcore.internal.camera.CameraManagerInternal
import org.firstinspires.ftc.robotcore.internal.system.AppUtil
import org.firstinspires.ftc.robotcore.internal.system.Deadline
import org.firstinspires.ftc.robotcore.internal.vuforia.externalprovider.CameraMode
import org.firstinspires.ftc.robotcore.internal.vuforia.externalprovider.FrameFormat
import org.openftc.easyopencv.OpenCvCameraException
import java.util.concurrent.CountDownLatch
import java.util.concurrent.Executor
import java.util.concurrent.TimeUnit

@TeleOp(name = "Tensorflow")
class TensorflowTest : OpMode() {

    private companion object {
        private const val WIDTH = 640
        private const val HEIGHT = 360
    }

    private val classifier by fastLazy { TFLiteClassifier(hardwareMap.appContext) }
    private lateinit var camera: Camera
    private lateinit var serialThreadPool: Executor
    private lateinit var cameraCaptureSession: CameraCaptureSession
    private var preview: PreviewSurfaceView? = null
    private var frameCount = 0

    override fun init() {
        val cameraManager = ClassFactory.getInstance().cameraManager as CameraManagerInternal
        this.serialThreadPool = cameraManager.serialThreadPool
        val cameraMonitorViewId = hardwareMap.appContext.resources.getIdentifier(
            "cameraMonitorViewId",
            "id",
            hardwareMap.appContext.packageName
        )
        val cameraName = hardwareMap.get(WebcamName::class.java, "Expensive Webcam")
        camera = cameraManager.requestPermissionAndOpenCamera(Deadline(10, TimeUnit.SECONDS), cameraName, null)

        val captureCallback = CameraCaptureSession.CaptureCallback { _, request, cameraFrame ->
            val tag = "Camera Callback"
            if (classifier.isInitialized) {
                Log.v(tag, "New Frame: $frameCount")
                ++frameCount

                val bitmap = request.createEmptyBitmap()
                cameraFrame.copyToBitmap(bitmap)

                preview?.update(bitmap)

                val resultsList = classifier.classify(bitmap).filter { it.confidence > 0.10f && it.title != "???" }

                resultsList.forEach { recognition ->
                    Log.v("Camera Callback", recognition.toString())
                }
            }
        }

        val captureStartResult = CountDownLatch(1)
        val stateCallback = object : CameraCaptureSession.StateCallback {
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
                            StatusCallback { _, cameraCaptureSequenceId, lastFrameNumber ->
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
                cameraCaptureSession = session
                captureStartResult.countDown()
            }

            override fun onClosed(session: CameraCaptureSession) = Unit
        }

        // Start the Camera
        camera.createCaptureSession(Continuation.create(serialThreadPool, stateCallback))

        captureStartResult.await(1, TimeUnit.SECONDS)
        classifier.initialize()
        telemetry.addData("Webcam", "Streaming started")
        telemetry.update()

        setupViewPort(cameraMonitorViewId)
    }

    private fun setupViewPort(containerLayoutId: Int) {
        AppUtil.getInstance().activity!!.runOnUiThread {
            val viewportContainerLayout: LinearLayout? =
                AppUtil.getInstance().activity!!.findViewById(containerLayoutId)

            if (viewportContainerLayout == null) {
                throw OpenCvCameraException("Viewport container specified by user does not exist!")
            } else if (viewportContainerLayout.childCount != 0) {
                throw OpenCvCameraException("Viewport container specified by user is not empty!")
            }

            preview = PreviewSurfaceView(AppUtil.getInstance().activity, null).apply {
                setSize(Size(WIDTH, HEIGHT))
                layoutParams = LinearLayout.LayoutParams(
                    ViewGroup.LayoutParams.MATCH_PARENT,
                    ViewGroup.LayoutParams.MATCH_PARENT
                )
                activate()
                resume()
            }

            viewportContainerLayout.visibility = View.VISIBLE
            viewportContainerLayout.addView(preview)
        }
    }

    override fun loop() {

    }

    override fun stop() {
        // Stop TFDetector
        classifier.close()

        // Stop the Preview
        preview?.let {
            it.pause()
            it.deactivate()
        }

        // Stop the Camera Stream and close the camera
        cameraCaptureSession.stopCapture()
        camera.close()
    }
}
