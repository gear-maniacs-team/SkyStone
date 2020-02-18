package net.gearmaniacs.teamcode.autonomous.paths

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import net.gearmaniacs.teamcode.autonomous.AbstractAuto
import net.gearmaniacs.teamcode.utils.PathAction
import net.gearmaniacs.teamcode.utils.PathPoint

@Autonomous(name = "BluePath")
class BluePath : AbstractAuto() {
    override val path = listOf(
        PathPoint(16.05597713532161, 65.2780776775411, 0.46666666865348816, action = PathAction.START_INTAKE),
        PathPoint(43.551334170408964, 119.84583283366432, 0.5433333516120911),
        PathPoint(
            6.918858287345373,
            65.21526744713078,
            1.6177778244018555,
            action = PathAction.STOP_INTAKE or PathAction.ATTACH_GRIPPER
        ),
        PathPoint(-95.83773279640718, 65.20472652478816, 1.5499999523162842),
        PathPoint(
            -189.2235808905075,
            79.68327129506503,
            3.1277778148651123,
            action = PathAction.ATTACH_FOUNDATION or PathAction.EXTEND_OUTTAKE
        ),
        PathPoint(-147.71750123748714, 77.29353778329528, 1.6177778244018555),
        PathPoint(
            -188.0561073816339,
            75.64756168858534,
            1.5788888931274414,
            action = PathAction.DETACH_FOUNDATION or PathAction.RELEASE_GRIPPER
        ),
        PathPoint(-80.56863386460489, 65.87060287477074, 1.515555500984192, action = PathAction.RETRACT_OUTTAKE)
    )
}

@Autonomous(name = "ActionFree")
class ActionFree : AbstractAuto() {
    override val path = listOf(
        PathPoint(
            cmX = 0.8989557879020221,
            cmY = 89.8457203984784,
            angle = 0.9344444274902344,
            action = PathAction.START_INTAKE
        ),
        PathPoint(
            cmX = 1.6068357312174466,
            cmY = 103.60984542976617,
            angle = 1.304444432258606,
            action = PathAction.STOP_INTAKE
        ),
        PathPoint(
            cmX = -31.205545071066588,
            cmY = 69.06623584270177,
            angle = 1.5655555725097656,
            action = PathAction.ATTACH_GRIPPER
        ),
        PathPoint(cmX = -208.09381226461238, cmY = 70.22979075537879, angle = 1.5655555725097656),
        PathPoint(
            cmX = -208.09381226461238,
            cmY = 70.22979075537879,
            angle = 3.1355555057525635,
            action = PathAction.ATTACH_FOUNDATION
        ),
        PathPoint(
            cmX = -158.54888344653173,
            cmY = 39.61074102689118,
            angle = 1.5722222328186035,
            action = PathAction.EXTEND_OUTTAKE
        ),
        PathPoint(
            cmX = -186.450179381556,
            cmY = 54.43925975068625,
            angle = 1.5922222137451172,
            action = PathAction.DETACH_FOUNDATION
        ),
        PathPoint(cmX = -86.8005347911151, cmY = 66.7361209479802, angle = 1.600000023841858)
    )
}