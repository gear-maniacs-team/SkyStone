package net.gearmaniacs.teamcode.utils

object Ranges {

    fun isRangeValid(number: Double, min: Double, max: Double) = min < number && number < max

    fun isRangeValid(number: Float, min: Float, max: Float) = min < number && number < max

    fun map(inputStart: Double, inputEnd: Double, outputStart: Double, outputEnd: Double, input: Double) =
        outputStart + ((outputEnd - outputStart) / (inputEnd - inputStart)) * (input - inputStart)
}
