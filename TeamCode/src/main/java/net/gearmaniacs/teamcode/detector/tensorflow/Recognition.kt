package net.gearmaniacs.teamcode.detector.tensorflow

import android.graphics.RectF
import kotlin.math.roundToInt

data class Recognition(
    val id: String,
    val title: String,
    val confidence: Float,
    val location: RectF
) {

    override fun toString(): String = buildString {
        append('[').append(id).append("], ")
        append(title).append(", ")
        append((confidence * 100f).roundToInt()).append("%, ")
        append(location.toString())
    }
}
