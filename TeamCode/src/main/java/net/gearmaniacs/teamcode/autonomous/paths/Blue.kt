package net.gearmaniacs.teamcode.autonomous.paths

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import net.gearmaniacs.teamcode.autonomous.AbstractAuto
import net.gearmaniacs.teamcode.utils.PathAction
import net.gearmaniacs.teamcode.utils.PathPoint

@Autonomous(name = "BluePath")
class BluePath : AbstractAuto() {
    override val path = listOf(
        PathPoint(-34.78447179328336, 5.695530944585069, -1.9787472251226996),
        PathPoint(-20.83762347421975, -22.44027411788463, -4.0740652579580505, action = PathAction.START_INTAKE),
        PathPoint(-10.074056428487, -3.48407367396716, 0.6404355124214304),
        PathPoint(-15.596172054751209, -2.2767761854385675, 6.564288019820091, action = PathAction.STOP_INTAKE),
        PathPoint(-44.59028953724461, -24.26896370664251, 9.380711942877959, action = PathAction.ATTACH_FOUNDATION or PathAction.ATTACH_GRIPPER),
        PathPoint(-39.01623414050167, -42.694786962663265, 8.831083400232771, action = PathAction.EXTEND_OUTTAKE),
        PathPoint(-50.2877964354901, -27.084968104781904, 10.17530812491952, action = PathAction.DETACH_FOUNDATION),
        PathPoint(-48.78479611506912, -14.950191629734631, 4.827270356101652)
    )
}