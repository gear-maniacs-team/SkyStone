package net.gearmaniacs.teamcode

import com.qualcomm.robotcore.eventloop.opmode.OpMode

abstract class TeamOpMode : OpMode() {

    protected val robot = TeamRobot()

    override fun start() {
        robot.start()
    }

    override fun stop() {
        robot.stop()
    }
}
