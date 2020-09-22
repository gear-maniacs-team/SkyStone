package net.gearmaniacs.teamcode.utils

import kotlin.math.max
import kotlin.math.min

object Ranges {

    fun clamp(number: Float, min: Float,max: Float) = min(max(number,min),max)

    fun isRangeValid(number: Double, min: Double, max: Double) = min < number && number < max

    fun isRangeValid(number: Float, min: Float, max: Float) = min < number && number < max

    fun map(inputStart: Double, inputEnd: Double, outputStart: Double, outputEnd: Double, input: Double) =
        outputStart + ((outputEnd - outputStart) / (inputEnd - inputStart)) * (input - inputStart)
}
