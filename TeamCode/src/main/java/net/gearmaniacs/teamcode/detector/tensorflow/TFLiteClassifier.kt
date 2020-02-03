package net.gearmaniacs.teamcode.detector.tensorflow

/*
import android.content.Context
import android.content.res.AssetManager
import android.graphics.Bitmap
import android.graphics.RectF
import org.tensorflow.lite.Interpreter
import java.io.FileInputStream
import java.io.IOException
import java.nio.ByteBuffer
import java.nio.ByteOrder
import java.nio.channels.FileChannel
import java.util.HashMap

class TFLiteClassifier(private val context: Context) {

    private val lock = Any()
    private var interpreter: Interpreter? = null
    @Volatile
    var isInitialized = false
        private set

    private var labels = emptyList<String>()

    private var inputImageWidth = 0
    private var inputImageHeight = 0
    private var modelInputSize = 0

    @Throws(IOException::class)
    fun initialize() {
        if (isInitialized) return

        val assetManager = context.assets
        val model = loadModelFile(assetManager)

        labels = loadLines(context)
        val options = Interpreter.Options()
        options.setNumThreads(2)

        val interpreter = Interpreter(model, options)

        val inputShape = interpreter.getInputTensor(0).shape()
        inputImageWidth = inputShape[1]
        inputImageHeight = inputShape[2]
        modelInputSize = inputImageWidth * inputImageHeight * inputShape[3]

        synchronized(lock) {
            this.interpreter = interpreter
            isInitialized = true
        }
    }

    @Throws(IOException::class)
    private fun loadModelFile(assetManager: AssetManager): ByteBuffer {
        val fileDescriptor = assetManager.openFd(TFLITE_FILE)
        val inputStream = FileInputStream(fileDescriptor.fileDescriptor)
        val fileChannel = inputStream.channel
        val declaredLength = fileDescriptor.declaredLength
        return fileChannel.map(FileChannel.MapMode.READ_ONLY, fileDescriptor.startOffset, declaredLength)
    }

    @Throws(IOException::class)
    private fun loadLines(context: Context): List<String> {
        val inputStream = context.assets.open(LABELS_FILE)
        inputStream.bufferedReader().use {
            return it.readLines()
        }
    }

    fun classify(bitmap: Bitmap): List<Recognition> {
        check(isInitialized) { "TF Lite Interpreter is not initialized yet." }
        val resizedImage = if (bitmap.width != inputImageWidth || bitmap.height != inputImageHeight)
            Bitmap.createScaledBitmap(bitmap, inputImageWidth, inputImageHeight, true)
        else bitmap

        val byteBuffer = convertBitmapToByteBuffer(resizedImage)

        val outputLocations = Array(1) { Array(NUM_DETECTIONS) { FloatArray(4) } }
        val outputClasses = Array(1) { FloatArray(NUM_DETECTIONS) }
        val outputScores = Array(1) { FloatArray(NUM_DETECTIONS) }
        val numDetections = FloatArray(1)

        val outputMap: MutableMap<Int, Any> = HashMap(4)
        outputMap[0] = outputLocations
        outputMap[1] = outputClasses
        outputMap[2] = outputScores
        outputMap[3] = numDetections

        synchronized(lock) {
            interpreter!!.runForMultipleInputsOutputs(arrayOf(byteBuffer), outputMap)
        }

        val list = ArrayList<Recognition>(5)
        val number = numDetections.first().toInt()

        for (i in 0 until number) {
            val location = RectF(
                outputLocations[0][i][1] * inputImageWidth,
                outputLocations[0][i][0] * inputImageHeight,
                outputLocations[0][i][3] * inputImageWidth,
                outputLocations[0][i][2] * inputImageHeight
            )
            val recognition = Recognition(
                i.toString(),
                labels[outputClasses[0][i].toInt()],
                outputScores[0][i],
                location
            )
            list.add(recognition)
        }

        return list
    }

    fun close() {
        synchronized(lock) {
            isInitialized = false
            interpreter?.close()
        }
    }

    private fun convertBitmapToByteBuffer(bitmap: Bitmap): ByteBuffer {
        val byteBuffer = ByteBuffer.allocateDirect(modelInputSize)
        byteBuffer.order(ByteOrder.nativeOrder())

        val pixels = IntArray(inputImageWidth * inputImageHeight)
        bitmap.getPixels(pixels, 0, bitmap.width, 0, 0, bitmap.width, bitmap.height)

        var pixel = 0
        for (i in 0 until inputImageWidth) {
            for (j in 0 until inputImageHeight) {
                val pixelVal = pixels[pixel++]

                // Quantized model
                byteBuffer.put((pixelVal shr 16 and 0xFF).toByte())
                byteBuffer.put((pixelVal shr 8 and 0xFF).toByte())
                byteBuffer.put((pixelVal and 0xFF).toByte())
            }
        }

        bitmap.recycle()

        return byteBuffer
    }

    companion object {
        private const val TFLITE_FILE = "CustomSkystone.tflite"
        private const val LABELS_FILE = "labelmap.txt"
        private const val NUM_DETECTIONS = 10
    }
}
*/
