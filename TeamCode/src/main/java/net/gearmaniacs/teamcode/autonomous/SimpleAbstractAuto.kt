package net.gearmaniacs.teamcode.autonomous

import net.gearmaniacs.teamcode.utils.PathPoint

abstract class SimpleAbstractAuto : AbstractAuto() {

    override val usesDetector = false

    final override val pathLeft = emptyList<PathPoint>()

    final override val pathCenter: List<PathPoint>
        get() = path

    final override val pathRight = emptyList<PathPoint>()

    abstract val path: List<PathPoint>
}
