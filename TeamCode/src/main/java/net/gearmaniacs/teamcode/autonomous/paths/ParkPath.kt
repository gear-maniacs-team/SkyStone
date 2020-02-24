package net.gearmaniacs.teamcode.autonomous.paths

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import net.gearmaniacs.teamcode.autonomous.SimpleAbstractAuto
import net.gearmaniacs.teamcode.utils.PathPoint

@Autonomous(name = "ParkToRight", group = "Park")
class ParkRight : SimpleAbstractAuto() {
    override val path = listOf(
        PathPoint(30.0, 0.0, 0.0)
    )
}

@Autonomous(name = "ParkToLeft", group = "Park")
class ParkLeft : SimpleAbstractAuto() {
    override val path = listOf(
        PathPoint(-30.0, 0.0, 0.0)
    )
}
