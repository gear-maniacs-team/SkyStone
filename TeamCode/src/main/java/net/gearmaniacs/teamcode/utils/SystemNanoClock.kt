package net.gearmaniacs.teamcode.utils

import android.os.SystemClock
import com.acmerobotics.roadrunner.util.NanoClock

object SystemNanoClock : NanoClock() {

    override fun seconds(): Double = SystemClock.elapsedRealtimeNanos() / 1e9
}
