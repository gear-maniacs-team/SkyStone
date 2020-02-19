package net.gearmaniacs.teamcode.autonomous.paths

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.Disabled
import net.gearmaniacs.teamcode.autonomous.AbstractAuto
import net.gearmaniacs.teamcode.utils.PathAction
import net.gearmaniacs.teamcode.utils.PathPoint

@Disabled
@Autonomous(name = "MotionProfilePath")
class MotionProfilePath : AbstractAuto() {

    override val path = listOf(
        PathPoint(40.0, 40.0, 0.0),
        PathPoint(20.0, 200.0, Math.PI),
        PathPoint(0.0, 0.0, 0.0)
    )
}
