package net.gearmaniacs.teamcode.autonomous.paths

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.Disabled
import net.gearmaniacs.teamcode.autonomous.SimpleAbstractAuto
import net.gearmaniacs.teamcode.utils.PathAction
import net.gearmaniacs.teamcode.utils.PathPoint

@Disabled
@Autonomous(name = "Foundation")
class FoundationPath : SimpleAbstractAuto() {
    override val path = listOf(
        PathPoint(0.0, 0.0, 0.0, action = PathAction.PREPARE_ATTACH_FOUNDATION),
        PathPoint(-20.0, -90.0, 0.0, action = PathAction.ATTACH_FOUNDATION or PathAction.SLEEP_500),
        PathPoint(10.0, -20.0, angle = Math.PI / 2, action = PathAction.DETACH_FOUNDATION),
        PathPoint(15.0, -10.0, angle = Math.PI / 2),
        PathPoint(85.0, -4.0, angle = Math.PI / 2)
    )
}
