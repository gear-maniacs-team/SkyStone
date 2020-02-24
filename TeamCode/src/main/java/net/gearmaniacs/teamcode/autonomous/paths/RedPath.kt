package net.gearmaniacs.teamcode.autonomous.paths

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import net.gearmaniacs.teamcode.autonomous.AbstractAuto
import net.gearmaniacs.teamcode.utils.PathAction
import net.gearmaniacs.teamcode.utils.PathPoint

@Autonomous(name = "RedPath", group = "Path")
class RedPath : AbstractAuto() {

    override val isBluePath = false

    override val pathLeft = listOf(
        PathPoint(
            x = -5.0,
            y = 100.0,
            angle = Math.toRadians(80.0),
            action = PathAction.START_INTAKE
        ),
        PathPoint(
            x = 6.0,
            y = 100.0,
            angle = Math.toRadians(85.0),
            action = PathAction.SLEEP_500
        ),
        PathPoint(
            x = -33.0,
            y = 76.0,
            angle = 1.558,
            action = PathAction.PREPARE_ATTACH_FOUNDATION
        ),
        PathPoint(
            x = -208.0,
            y = 65.0,
            angle = 3.135,
            moveError = 3.5,
            turnError = Math.toRadians(15.0),
            action = PathAction.STOP_INTAKE or PathAction.ATTACH_GRIPPER
        ),
        PathPoint(
            x = -208.0,
            y = 90.0,
            angle = 3.135,
            turnError = Math.toRadians(10.0),
            action = PathAction.ATTACH_FOUNDATION or PathAction.EXTEND_OUTTAKE
        ),
        PathPoint(
            x = -161.6,
            y = 58.09,
            angle = 1.558,
            moveError = 3.0,
            turnError = Math.toRadians(10.0),
            action = PathAction.DETACH_FOUNDATION or PathAction.RETRACT_OUTTAKE
        ),
        PathPoint(
            x = 43.0,
            y = 74.0,
            angle = 1.558,
            moveError = 2.5,
            action = PathAction.START_INTAKE
        ),
        PathPoint(
            x = 45.0,
            y = 100.0,
            angle = 1.558 - Math.toRadians(15.0)
        ),
        PathPoint(
            x = 46.0,
            y = 100.0,
            angle = 1.558 - Math.toRadians(10.0),
            action = PathAction.SLEEP_500
        ),
        PathPoint(
            x = -60.0,
            y = 72.0,
            angle = 1.558,
            moveError = 3.5,
            action = PathAction.STOP_INTAKE or PathAction.ATTACH_GRIPPER
        ),
        PathPoint(
            x = -180.0,
            y = 80.0,
            angle = 1.558,
            moveError = 4.5,
            action = PathAction.EXTEND_OUTTAKE
        ),
        PathPoint(
            x = -190.0,
            y = 80.0,
            angle = 1.558,
            moveError = 3.0,
            action = PathAction.SLEEP_500 or PathAction.RELEASE_GRIPPER or PathAction.RETRACT_OUTTAKE
        ),
        PathPoint(
            x = -80.0,
            y = 70.0,
            angle = 1.558
        )
    )

    override val pathCenter = listOf(
        PathPoint(
            x = -28.0,
            y = 98.0,
            angle = 1.558 - Math.toRadians(15.0),
            action = PathAction.START_INTAKE
        ),
        PathPoint(
            x = -16.0,
            y = 98.0,
            angle = 1.558 - Math.toRadians(15.0),
            action = PathAction.SLEEP_500
        ),
        PathPoint(
            x = -31.0,
            y = 80.0,
            angle = 1.558,
            action = PathAction.PREPARE_ATTACH_FOUNDATION
        ),
        PathPoint(
            x = -208.0,
            y = 75.0,
            angle = 3.135,
            moveError = 3.5,
            turnError = Math.toRadians(10.0),
            action = PathAction.STOP_INTAKE or PathAction.ATTACH_GRIPPER
        ),
        PathPoint(
            x = -208.0,
            y = 90.0,
            angle = 3.135,
            action = PathAction.ATTACH_FOUNDATION or PathAction.EXTEND_OUTTAKE
        ),
        PathPoint(
            x = -155.0,
            y = 68.0,
            angle = 1.558,
            moveError = 3.0,
            turnError = Math.toRadians(8.0),
            action = PathAction.DETACH_FOUNDATION or PathAction.RETRACT_OUTTAKE
        ),
        PathPoint(
            x = 23.0,
            y = 76.0,
            angle = 1.558,
            moveError = 2.5,
            action = PathAction.START_INTAKE
        ),
        PathPoint(
            x = 41.0,
            y = 110.0,
            angle = 1.558 - Math.toRadians(20.0),
            action = PathAction.SLEEP_500
        ),
        PathPoint(
            x = -60.0,
            y = 72.0,
            angle = 1.558,
            moveError = 3.5,
            action = PathAction.STOP_INTAKE or PathAction.ATTACH_GRIPPER
        ),
        PathPoint(
            x = -180.0,
            y = 80.0,
            angle = 1.558,
            moveError = 4.5,
            action = PathAction.EXTEND_OUTTAKE
        ),
        PathPoint(
            x = -190.0,
            y = 80.0,
            angle = 1.558,
            moveError = 3.0,
            action = PathAction.SLEEP_500 or PathAction.RELEASE_GRIPPER or PathAction.RETRACT_OUTTAKE
        ),
        PathPoint(
            x = -80.0,
            y = 70.0,
            angle = 1.558
        )
    )

    override val pathRight = listOf(
        PathPoint(
            x = -40.0,
            y = 80.0,
            angle = Math.toRadians(45.0),
            action = PathAction.START_INTAKE
        ),
        PathPoint(
            x = -26.0,
            y = 93.0,
            moveError = 1.5,
            angle = Math.toRadians(35.0),
            action = PathAction.SLEEP_500
        ),
        PathPoint(
            x = -32.0,
            y = 75.0,
            angle = 1.558,
            action = PathAction.PREPARE_ATTACH_FOUNDATION
        ),
        PathPoint(
            x = -208.0,
            y = 75.0,
            angle = 3.135,
            moveError = 4.0,
            action = PathAction.STOP_INTAKE or PathAction.ATTACH_GRIPPER,
            turnError = Math.toRadians(12.0)
        ),
        PathPoint(
            x = -208.0,
            y = 90.0,
            angle = 3.135,
            action = PathAction.ATTACH_FOUNDATION or PathAction.EXTEND_OUTTAKE,
            turnError = Math.toRadians(15.0)
        ),
        PathPoint(
            x = -161.6,
            y = 58.09,
            angle = 1.558,
            moveError = 3.0,
            turnError = Math.toRadians(10.0),
            action = PathAction.DETACH_FOUNDATION or PathAction.RETRACT_OUTTAKE
        ),
        PathPoint(
            x = 10.0,
            y = 72.0,
            angle = 1.558,
            moveError = 2.5,
            action = PathAction.START_INTAKE
        ),
        PathPoint(
            x = 16.0,
            y = 110.0,
            angle = 1.558 - Math.toRadians(15.0)
        ),
        PathPoint(
            x = 20.0,
            y = 110.0,
            angle = 1.558 - Math.toRadians(15.0),
            action = PathAction.SLEEP_500
        ),
        PathPoint(
            x = -60.0,
            y = 72.0,
            angle = 1.558,
            moveError = 3.5,
            action = PathAction.STOP_INTAKE or PathAction.ATTACH_GRIPPER
        ),
        PathPoint(
            x = -190.0,
            y = 78.0,
            angle = 1.558,
            moveError = 4.0,
            action = PathAction.EXTEND_OUTTAKE
        ),
        PathPoint(
            x = -190.0,
            y = 80.0,
            angle = 1.558,
            moveError = 4.0,
            action = PathAction.RELEASE_GRIPPER or PathAction.RETRACT_OUTTAKE
        ),
        PathPoint(
            x = -80.0,
            y = 74.0,
            angle = 1.558
        )
    )
}
