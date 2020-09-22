package net.gearmaniacs.teamcode.utils.extensions

import net.gearmaniacs.teamcode.utils.MathUtils
import kotlin.math.abs
import kotlin.math.sign

const val CM_TO_INCH = 0.394

infix fun Double.epsilonEquals(other: Double) = abs(this - other) < MathUtils.EPSILON

infix fun Float.epsilonEquals(other: Float) = abs(this - other) < MathUtils.EPSILON

fun Long.millisToSeconds() = this / 1000.0f

fun Double.coerceRange(minimumValue: Double): Double {
    return if (abs(this) < minimumValue) sign(this) * minimumValue else this
}
