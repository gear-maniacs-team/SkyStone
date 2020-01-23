/*
package org.firstinspires.ftc.teamcode.detector

import android.content.Context
import android.content.res.AssetManager
import android.graphics.Bitmap
import android.graphics.RectF
import android.util.Log
import org.tensorflow.lite.Interpreter
import java.io.FileInputStream
import java.io.IOException
import java.io.InputStreamReader
import java.nio.BufferOverflowException
import java.nio.ByteBuffer
import java.nio.ByteOrder
import java.nio.channels.FileChannel
import java.util.*
import kotlin.collections.ArrayList

class TFLiteClassifier(private val context: Context) {

    private var interpreter: Interpreter? = null
    var isInitialized = false
        private set

    private var labels = ArrayList<String>()

    private var inputImageWidth = 0
    private var inputImageHeight = 0
    private var modelInputSize = 0

    @Throws(IOException::class)
    fun initialize() {
        val assetManager = context.assets
        val model = loadModelFile(assetManager, "CustomSkystone.tflite")

        labels = loadLines(context, "labelmap.txt")
        val options = Interpreter.Options()
        val interpreter = Interpreter(model, options)

        val inputShape = interpreter.getInputTensor(0).shape()
        inputImageWidth = inputShape[1]
        inputImageHeight = inputShape[2]
        modelInputSize = inputImageWidth * inputImageHeight * inputShape[3]

        this.interpreter = interpreter

        isInitialized = true
    }

    @Throws(IOException::class)
    private fun loadModelFile(assetManager: AssetManager, fileName: String): ByteBuffer {
        val fileDescriptor = assetManager.openFd(fileName)
        val inputStream = FileInputStream(fileDescriptor.fileDescriptor)
        val fileChannel = inputStream.channel
        val startOffset = fileDescriptor.startOffset
        val declaredLength = fileDescriptor.declaredLength
        return fileChannel.map(FileChannel.MapMode.READ_ONLY, startOffset, declaredLength)
    }

    @Throws(IOException::class)
    fun loadLines(context: Context, filename: String): ArrayList<String> {
        val s = Scanner(InputStreamReader(context.assets.open(filename)))
        val labels = ArrayList<String>()
        while (s.hasNextLine()) {
            labels.add(s.nextLine())
        }
        s.close()
        return labels
    }

    private fun getMaxResult(result: FloatArray): Int {
        var probability = result[0]
        var index = 0
        for (i in result.indices) {
            if (probability < result[i]) {
                probability = result[i]
                index = i
            }
        }
        return index
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

        interpreter!!.runForMultipleInputsOutputs(arrayOf(byteBuffer), outputMap)
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
        interpreter?.close()
    }

    private fun convertBitmapToByteBuffer(bitmap: Bitmap): ByteBuffer {
        val byteBuffer = ByteBuffer.allocateDirect(modelInputSize)
        byteBuffer.order(ByteOrder.nativeOrder())

        val pixels = IntArray(inputImageWidth * inputImageHeight)
        bitmap.getPixels(pixels, 0, bitmap.width, 0, 0, bitmap.width, bitmap.height)
        var pixel = 0
        try {
            for (i in 0 until inputImageWidth) {
                for (j in 0 until inputImageHeight) {
                    val pixelVal = pixels[pixel++]

                    // Quantized model
                    byteBuffer.put((pixelVal shr 16 and 0xFF).toByte())
                    byteBuffer.put((pixelVal shr 8 and 0xFF).toByte())
                    byteBuffer.put((pixelVal and 0xFF).toByte())
                }
            }
        } catch (e: BufferOverflowException) {
            Log.e(TAG, "Pixel: $pixel")
            e.printStackTrace()
        }

        bitmap.recycle()

        return byteBuffer
    }

    companion object {
        private const val TAG = "TFLiteClassifier"
        private const val NUM_DETECTIONS = 5
    }
}
*/