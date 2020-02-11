package net.gearmaniacs.teamcode.utils

import android.os.SystemClock
import com.acmerobotics.roadrunner.util.NanoClock

/**
 * A [NanoClock] implementation backed by [SystemClock]
 */
object RobotClock : NanoClock() {

    fun nano() = SystemClock.elapsedRealtimeNanos()

    fun millis() = SystemClock.elapsedRealtime()

    override fun seconds(): Double = SystemClock.elapsedRealtimeNanos() / 1e9
}
