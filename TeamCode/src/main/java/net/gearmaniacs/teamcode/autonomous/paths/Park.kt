package net.gearmaniacs.teamcode.autonomous.paths

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import net.gearmaniacs.teamcode.autonomous.AbstractAuto
import net.gearmaniacs.teamcode.utils.PathPoint

@Autonomous(name = "ParkToRight", group = "Park")
class ParkRight : AbstractAuto() {
    override val usesDetector = false
    override val pathLeft = emptyList<PathPoint>()
    override val pathCenter = listOf(
        PathPoint(30.0, 0.0, 0.0)
    )
    override val pathRight = emptyList<PathPoint>()
}

@Autonomous(name = "ParkToLeft", group = "Park")
class ParkLeft : AbstractAuto() {
    override val usesDetector = false
    override val pathLeft = emptyList<PathPoint>()
    override val pathCenter = listOf(
        PathPoint(-30.0, 0.0, 0.0)
    )
    override val pathRight = emptyList<PathPoint>()
}
