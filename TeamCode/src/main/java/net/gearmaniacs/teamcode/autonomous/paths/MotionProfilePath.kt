package net.gearmaniacs.teamcode.autonomous.paths

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import net.gearmaniacs.teamcode.autonomous.AbstractAuto
import net.gearmaniacs.teamcode.utils.PathAction
import net.gearmaniacs.teamcode.utils.PathPoint

@Autonomous(name = "AutoPath")
class MotionProfilePath : AbstractAuto() {

    override val path = listOf(
        /*PathPoint(0.0, -60.0, 0.0, action = PathAction.ATTACH_FOUNDATION),
        PathPoint(-10.0, -20.0, Math.PI / 2, action = PathAction.DETACH_FOUNDATION),
        PathPoint(-20.0, -20.0, Math.PI / 2),
        PathPoint(60.0, -20.0, Math.PI / 2)*/
        PathPoint(40.0, 40.0, 0.0),
        PathPoint(20.0, 200.0, Math.PI),
        PathPoint(0.0, 0.0, 0.0)
    )
}
