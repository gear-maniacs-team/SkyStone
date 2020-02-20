package net.gearmaniacs.teamcode.autonomous.paths

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import net.gearmaniacs.teamcode.autonomous.AbstractAuto
import net.gearmaniacs.teamcode.utils.PathAction
import net.gearmaniacs.teamcode.utils.PathPoint

@Autonomous(name = "BluePath")
class BluePath : AbstractAuto() {

    override val pathRight = listOf(
        PathPoint(
            x = 1.0,
            y = 90.0,
            angle = 0.9345,
            action = PathAction.START_INTAKE,
            moveError = 3.0
        ),
        PathPoint(
            x = -31.0,
            y = 69.0,
            angle = 1.558,
            action = PathAction.PREPARE_ATTACH_FOUNDATION
        ),
        PathPoint(
            x = -208.0,
            y = 65.0,
            angle = 3.135,
            moveError = 3.5,
            action = PathAction.STOP_INTAKE or PathAction.ATTACH_GRIPPER
        ),
        PathPoint(
            x = -208.0,
            y = 90.0,
            angle = 3.135,
            action = PathAction.ATTACH_FOUNDATION or PathAction.EXTEND_OUTTAKE
        ),
        PathPoint(
            x = -161.6,
            y = 58.09,
            angle = 1.558,
            moveError = 3.0,
            action = PathAction.DETACH_FOUNDATION or PathAction.RETRACT_OUTTAKE
        ),
        PathPoint(
            x = 43.0,
            y = 76.0,
            angle = 1.558,
            moveError = 2.5,
            action = PathAction.START_INTAKE
        ),
//        PathPoint(
//            x = 43.0,
//            y = 109.0,
//            angle = 1.558,
//            moveError = 4.0
//        ),
        PathPoint(
            x = 55.0,
            y = 110.0,
            angle = 1.558 - Math.toRadians(15.0),
            moveError = 2.5
        ),
        PathPoint(
            x = -60.0,
            y = 74.0,
            angle = 1.558,
            moveError = 3.5,
            turnError = Math.toRadians(10.0),
            action = PathAction.STOP_INTAKE or PathAction.ATTACH_GRIPPER
        ),
        PathPoint(
            x = -190.0,
            y = 80.0,
            angle = 1.558,
            moveError = 3.5,
            action = PathAction.STOP_INTAKE or PathAction.ATTACH_GRIPPER or PathAction.EXTEND_OUTTAKE
        ),
        PathPoint(
            x = -190.0,
            y = 80.0,
            angle = 1.558,
            moveError = 4.0,
            action = PathAction.SLEEP_500 or PathAction.RELEASE_GRIPPER or PathAction.RETRACT_OUTTAKE
        ),
        PathPoint(
            x = -80.0,
            y = 70.0,
            angle = 1.558
        )
    )
}
