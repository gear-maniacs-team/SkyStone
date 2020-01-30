package net.gearmaniacs.teamcode.detector

import android.content.Context
import android.graphics.Bitmap
import android.graphics.Color
import android.graphics.Rect
import android.util.Log
import android.view.SurfaceHolder
import android.view.SurfaceView
import org.firstinspires.ftc.robotcore.external.android.util.Size
import org.firstinspires.ftc.robotcore.internal.collections.EvictingBlockingQueue
import java.util.concurrent.ArrayBlockingQueue
import kotlin.math.roundToInt

class PreviewSurfaceView(
    context: Context?,
    onClickListener: OnClickListener?
) : SurfaceView(context), SurfaceHolder.Callback {

    private var size = Size(0, 0)
    private var aspectRatio = 0.0
    private var renderThread: RenderThread? = null
    private val visionPreviewFrameQueue = EvictingBlockingQueue(ArrayBlockingQueue<Bitmap>(4))
    @Volatile
    private var internalRenderingState = RenderingState.STOPPED
    private val syncObj = Any()
    @Volatile
    private var userRequestedActive = false
    @Volatile
    private var userRequestedPause = false
    private var needToDeactivateRegardlessOfUser = false
    private var surfaceExistsAndIsReady = false

    private enum class RenderingState {
        STOPPED, ACTIVE, PAUSED
    }

    init {
        holder.addCallback(this)
        visionPreviewFrameQueue.setEvictAction { obj: Bitmap -> obj.recycle() }
        setOnClickListener(onClickListener)
    }

    fun setSize(size: Size) {
        synchronized(syncObj) {
            renderThread?.interrupt()
            this.size = size
            aspectRatio = size.width.toDouble() / size.height.toDouble()
        }
    }

    fun update(bitmap: Bitmap) {
        synchronized(syncObj) {
            // Are we actually rendering to the display right now?
            if (internalRenderingState == RenderingState.ACTIVE) {
                // Copy this Bitmap as it might be copied in the future
                visionPreviewFrameQueue.offer(bitmap.copy(Bitmap.Config.ARGB_8888, false))
            }
        }
    }

    /*
     * Called with syncObj held
     */
    fun checkState() {
        // If the surface isn't ready, don't do anything
        if (!surfaceExistsAndIsReady) {
            Log.d(TAG, "CheckState(): surface not ready or doesn't exist")
            return
        }
        // Does the user want us to stop?
        if (!userRequestedActive || needToDeactivateRegardlessOfUser) {
            Log.d(TAG, "CheckState(): user requested that we deactivate")
            /*
             * We only need to stop the render thread if it's not
             * already stopped
             */
            if (internalRenderingState != RenderingState.STOPPED) {
                Log.d(TAG, "CheckState(): deactivating viewport")
                /*
                 * Interrupt him so he's not stuck looking at his
                 * frame queue.
                 */
                val renderThread = renderThread!!
                renderThread.notifyExitRequested()
                renderThread.interrupt()
                try {
                    renderThread.join()
                } catch (e: InterruptedException) {
                    e.printStackTrace()
                }
                internalRenderingState = RenderingState.STOPPED
            } else {
                Log.d(TAG, "CheckState(): already deactivated")
            }
        } else if (userRequestedActive) {
            Log.d(TAG, "CheckState(): user requested that we activate")
            /*
             * We only need to start the render thread if it's
             * stopped.
             */
            if (internalRenderingState == RenderingState.STOPPED) {
                Log.d(TAG, "CheckState(): activating viewport")
                internalRenderingState = RenderingState.PAUSED
                internalRenderingState = if (userRequestedPause) {
                    RenderingState.PAUSED
                } else {
                    RenderingState.ACTIVE
                }
                renderThread = RenderThread().apply {
                    start()
                }
            } else {
                Log.d(TAG, "CheckState(): already activated")
            }
        }
        if (internalRenderingState != RenderingState.STOPPED) {
            if (userRequestedPause && internalRenderingState != RenderingState.PAUSED
                || !userRequestedPause && internalRenderingState != RenderingState.ACTIVE
            ) {
                internalRenderingState = if (userRequestedPause) {
                    Log.d(TAG, "CheckState(): pausing viewport")
                    RenderingState.PAUSED
                } else {
                    Log.d(TAG, "CheckState(): resuming viewport")
                    RenderingState.ACTIVE
                }
                /*
                 * Interrupt him so that he's not stuck looking at his frame queue.
                 * (We stop filling the frame queue if the user requested pause so
                 * we aren't doing pointless memcpys)
                 */
                renderThread!!.interrupt()
            }
        }
    }

    /**
     * Activate the render thread
     */
    fun activate() {
        synchronized(syncObj) {
            userRequestedActive = true
            checkState()
        }
    }

    /**
     * Deactivate the render thread
     */
    fun deactivate() {
        synchronized(syncObj) {
            userRequestedActive = false
            checkState()
        }
    }

    fun resume() {
        synchronized(syncObj) {
            userRequestedPause = false
            checkState()
        }
    }

    fun pause() {
        synchronized(syncObj) {
            userRequestedPause = true
            checkState()
        }
    }

    override fun surfaceCreated(holder: SurfaceHolder) = Unit

    override fun surfaceChanged(holder: SurfaceHolder, format: Int, width: Int, height: Int) {
        synchronized(syncObj) {
            needToDeactivateRegardlessOfUser = false
            surfaceExistsAndIsReady = true
            checkState()
        }
    }

    override fun surfaceDestroyed(holder: SurfaceHolder) {
        synchronized(syncObj) {
            needToDeactivateRegardlessOfUser = true
            checkState()
            surfaceExistsAndIsReady = false
        }
    }

    internal inner class RenderThread : Thread() {
        private var shouldPaintOrange = true
        @Volatile
        private var exitRequested = false

        fun notifyExitRequested() {
            exitRequested = true
        }

        override fun run() {
            Log.d(RENDER_THREAD_TAG, "I am alive!")
            var canvas = holder.lockCanvas()
            canvas!!.drawColor(Color.BLUE)
            holder.unlockCanvasAndPost(canvas)
            infiniteLoop@ while (true) {
                /*
                 * Do we need to exit?
                 */
                if (exitRequested)
                    break

                when (internalRenderingState) {
                    RenderingState.ACTIVE -> {
                        shouldPaintOrange = true
                        val bitmap = try { // Grab a Bitmap from the queue
                            visionPreviewFrameQueue.take()
                        } catch (e: InterruptedException) {
                            e.printStackTrace()
                            break@infiniteLoop
                        }
                        // Get canvas object for rendering on
                        canvas = holder.lockCanvas()
                        if (canvas != null) { // Draw the background black each time to prevent double buffering problems
                            canvas.drawColor(Color.BLACK)
                            // Landscape
                            if (canvas.height * aspectRatio < canvas.width) { // Draw the bitmap, scaling it to the maximum size that will fit in the viewport
                                canvas.drawBitmap(
                                    bitmap,
                                    Rect(0, 0, bitmap.width, bitmap.height),
                                    Rect(
                                        0,
                                        0,
                                        (canvas.height * aspectRatio).roundToInt(),
                                        canvas.height
                                    ),
                                    null
                                )
                            } else { // Draw the bitmap, scaling it to the maximum size that will fit in the viewport
                                canvas.drawBitmap(
                                    bitmap,
                                    Rect(0, 0, bitmap.width, bitmap.height),
                                    Rect(
                                        0,
                                        0,
                                        canvas.width,
                                        (canvas.width / aspectRatio).roundToInt()
                                    ),
                                    null
                                )
                            }
                            holder.unlockCanvasAndPost(canvas)
                        } else {
                            Log.d(TAG, "Canvas was null")
                        }

                        bitmap.recycle()
                    }
                    RenderingState.PAUSED -> {
                        if (shouldPaintOrange) {
                            shouldPaintOrange = false
                            canvas = holder.lockCanvas()

                            if (canvas != null) {
                                canvas.drawColor(Color.rgb(255, 166, 0))
                                holder.unlockCanvasAndPost(canvas)
                            }
                        }
                        try {
                            sleep(50)
                        } catch (e: InterruptedException) {
                            e.printStackTrace()
                        }
                    }
                    else -> Unit
                }
            }
            Log.d(RENDER_THREAD_TAG, "About to exit")
        }
    }

    companion object {
        private const val TAG = "PreviewViewport"
        private const val RENDER_THREAD_TAG = "PreviewRenderThread"
    }
}
