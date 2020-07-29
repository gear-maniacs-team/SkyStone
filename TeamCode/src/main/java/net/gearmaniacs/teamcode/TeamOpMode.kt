package net.gearmaniacs.teamcode

import android.support.annotation.CallSuper
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import net.gearmaniacs.teamcode.utils.IHardware
import net.gearmaniacs.teamcode.utils.IUpdatable

abstract class TeamOpMode : OpMode() {

    protected open val robot = TeamRobot()

    protected fun initRobot(
        hardwareList: List<IHardware> = emptyList(),
        updatableList: List<IUpdatable> = emptyList()
    ) {
        robot.init(hardwareMap, hardwareList, updatableList)
    }

    @CallSuper
    override fun start() {
        robot.start()
    }

    @CallSuper
    override fun stop() {
        robot.stop()
    }
}
