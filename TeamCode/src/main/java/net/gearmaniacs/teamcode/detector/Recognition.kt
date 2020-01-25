package net.gearmaniacs.teamcode.detector

import android.graphics.RectF

data class Recognition(
    val id: String?,
    val title: String?,
    val confidence: Float?,
    var location: RectF?
) {

    override fun toString(): String {
        var resultString = ""
        if (id != null)
            resultString += "[$id] "
        if (title != null)
            resultString += "$title "
        if (confidence != null)
            resultString += String.format("(%.1f%%) ", confidence * 100.0f)
        if (location != null)
            resultString += location.toString()
        return resultString.trim()
    }
}
