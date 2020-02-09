package net.gearmaniacs.teamcode

import android.support.annotation.CallSuper
import com.qualcomm.robotcore.eventloop.opmode.OpMode

abstract class TeamOpMode : OpMode() {

    protected val robot = TeamRobot()

    @CallSuper
    override fun start() {
        robot.start()
    }

    @CallSuper
    override fun stop() {
        robot.stop()
    }
}
