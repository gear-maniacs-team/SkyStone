package net.gearmaniacs.teamcode.autonomous.paths

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import net.gearmaniacs.teamcode.autonomous.AbstractAuto
import net.gearmaniacs.teamcode.utils.PathAction
import net.gearmaniacs.teamcode.utils.PathPoint

@Autonomous(name = "Foundation")
class Foundation : AbstractAuto() {
    override val usesDetector: Boolean
        get() = false
    override val pathLeft: List<PathPoint>
        get() = emptyList()
    override val pathCenter = listOf(
        PathPoint(0.0,0.0, 0.0, action = PathAction.PREPARE_ATTACH_FOUNDATION),
        PathPoint(-20.0, -90.0, 0.0, action = PathAction.ATTACH_FOUNDATION or PathAction.SLEEP_500),
        PathPoint(10.0, -20.0, angle = Math.PI / 2, action = PathAction.DETACH_FOUNDATION),
        PathPoint(15.0, -10.0, angle = Math.PI / 2),
        PathPoint(85.0, -4.0, angle = Math.PI / 2)
    )
    override val pathRight: List<PathPoint>
        get() = emptyList()
}