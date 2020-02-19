package net.gearmaniacs.teamcode.autonomous.paths

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import net.gearmaniacs.teamcode.autonomous.AbstractAuto
import net.gearmaniacs.teamcode.utils.PathAction
import net.gearmaniacs.teamcode.utils.PathPoint

@Autonomous(name = "BluePath")
class BluePath : AbstractAuto() {

    override val path = listOf(
        PathPoint(
            x = 1.0,
            y = 90.0,
            angle = 0.9344444274902344,
            action = PathAction.START_INTAKE
        ),
        PathPoint(
            x = 1.5,
            y = 101.0,
            angle = Math.PI / 2 //1.354444432258606
        ),
        PathPoint(
            x = -31.0,
            y = 69.0,
            angle = 1.5655555725097656,
            action = PathAction.STOP_INTAKE or PathAction.ATTACH_GRIPPER
        ),
        PathPoint(
            x = -208.0,
            y = 72.0,
            angle = 3.135
        ),
        PathPoint(
            x = -208.0,
            y = 90.0,
            angle = 3.135,
            action = PathAction.ATTACH_FOUNDATION
        ),
        PathPoint(
            x = -161.6,
            y = 58.09,
            angle = 1.558,
            moveError = 3.0,
            action = PathAction.EXTEND_OUTTAKE
        ),
        PathPoint(
            x = -190.0,
            y = 50.0,
            angle = 1.558,
            moveError = 3.5,
            action = PathAction.RELEASE_GRIPPER or PathAction.DETACH_FOUNDATION or PathAction.RETRACT_OUTTAKE
        ),
        PathPoint(
            x = -80.0,
            y = 70.0,
            angle = 1.558
        )
    )
}
