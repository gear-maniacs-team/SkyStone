package net.gearmaniacs.teamcode.autonomous

import android.graphics.Bitmap
import android.util.Log
import android.view.View
import android.view.ViewGroup
import android.widget.LinearLayout
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.util.RobotLog
import net.gearmaniacs.teamcode.detector.tensorflow.PreviewSurfaceView
import net.gearmaniacs.teamcode.detector.tensorflow.TFLiteClassifier
import net.gearmaniacs.teamcode.utils.fastLazy
import org.firstinspires.ftc.robotcore.external.ClassFactory
import org.firstinspires.ftc.robotcore.external.android.util.Size
import org.firstinspires.ftc.robotcore.external.function.Consumer
import org.firstinspires.ftc.robotcore.external.function.Continuation
import org.firstinspires.ftc.robotcore.external.hardware.camera.Camera
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraCaptureSession
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraCaptureSession.StatusCallback
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraException
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName
import org.firstinspires.ftc.robotcore.external.stream.CameraStreamSource
import org.firstinspires.ftc.robotcore.internal.camera.CameraManagerInternal
import org.firstinspires.ftc.robotcore.internal.system.AppUtil
import org.firstinspires.ftc.robotcore.internal.system.Deadline
import org.firstinspires.ftc.robotcore.internal.vuforia.externalprovider.CameraMode
import org.firstinspires.ftc.robotcore.internal.vuforia.externalprovider.FrameFormat
import java.util.concurrent.CountDownLatch
import java.util.concurrent.TimeUnit
import java.util.concurrent.atomic.AtomicReference

@TeleOp(name = "Tensorflow")
class TensorflowTest : OpMode(), CameraStreamSource {

    private companion object {
        private const val CAMERA_NAME = "Expensive Webcam"
        private const val CAMERA_WIDTH = 640
        private const val CAMERA_HEIGHT = 360
        private const val CAMERA_FPS = 20
    }

    private val classifier by fastLazy {
        TFLiteClassifier(
            hardwareMap.appContext
        )
    }
    private val latestBitmap = AtomicReference<Pair<Int, Bitmap>>()
    private var cameraMonitorViewId = 0
    private lateinit var camera: Camera
    private lateinit var cameraCaptureSession: CameraCaptureSession
    private var preview: PreviewSurfaceView? = null
    private var frameCount = 0

    override fun init() {
        cameraMonitorViewId = hardwareMap.appContext.resources.getIdentifier(
            "cameraMonitorViewId",
            "id",
            hardwareMap.appContext.packageName
        )
        val cameraManager = ClassFactory.getInstance().cameraManager as CameraManagerInternal
        val serialThreadPool = cameraManager.serialThreadPool
        val cameraName = hardwareMap.get(WebcamName::class.java, CAMERA_NAME)
        camera = cameraManager.requestPermissionAndOpenCamera(Deadline(10, TimeUnit.SECONDS), cameraName, null)

        val captureCallback = CameraCaptureSession.CaptureCallback { _, request, cameraFrame ->
            val tag = "Camera Callback"
            if (classifier.isInitialized) {
                Log.v(tag, "New Frame: $frameCount")

                val bitmap = request.createEmptyBitmap()
                cameraFrame.copyToBitmap(bitmap)

                preview?.update(bitmap)
                latestBitmap.set(Pair(frameCount, bitmap))
                ++frameCount
                Thread.yield()
            }
        }

        val captureStartResult = CountDownLatch(1)
        val stateCallback = object : CameraCaptureSession.StateCallback {
            override fun onConfigured(session: CameraCaptureSession) {
                try {
                    val streamingMode = CameraMode(
                        CAMERA_WIDTH,
                        CAMERA_HEIGHT,
                        CAMERA_FPS,
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

        setupViewPort()
    }

    private fun setupViewPort() {
        val activity = AppUtil.getInstance().activity!!
        activity.runOnUiThread {
            val viewportContainerLayout = activity.findViewById<LinearLayout>(cameraMonitorViewId)!!

            check(viewportContainerLayout.childCount == 0) { "Preview container specified by user is not empty!" }

            preview = PreviewSurfaceView(
                AppUtil.getInstance().activity,
                null
            ).apply {
                setSize(Size(CAMERA_WIDTH, CAMERA_HEIGHT))
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
        val tag = "Recognition"

        val pair = latestBitmap.getAndSet(null) ?: return
        val resultsList = classifier.classify(pair.second).filter { it.confidence > 0.10f && it.title != "???" }

        Log.v(tag, "Analyzed Frame number ${pair.first}")
        resultsList.forEach { recognition ->
            Log.v(tag, recognition.toString())
        }
    }

    override fun stop() {
        // Stop TFDetector
        classifier.close()

        // Stop the Preview
        preview?.let {
            it.pause()
            it.deactivate()

            val activity = AppUtil.getInstance().activity!!
            activity.runOnUiThread {
                val viewportContainerLayout = activity.findViewById<LinearLayout>(cameraMonitorViewId)!!
                viewportContainerLayout.removeAllViews()
            }
        }

        // Stop the Camera Stream
        cameraCaptureSession.stopCapture()
    }

    // For the Camera Preview
    override fun getFrameBitmap(continuation: Continuation<out Consumer<Bitmap>>?) {
        continuation?.dispatch { bitmapConsumer ->
            bitmapConsumer.accept(latestBitmap.get().second)
        }
    }
}
