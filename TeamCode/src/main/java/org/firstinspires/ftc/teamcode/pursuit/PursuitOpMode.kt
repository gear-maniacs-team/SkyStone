package org.firstinspires.ftc.teamcode.pursuit

import com.qualcomm.robotcore.eventloop.opmode.OpMode
import org.firstinspires.ftc.teamcode.RobotPos
import org.firstinspires.ftc.teamcode.TeamRobot

class PursuitOpMode : OpMode() {

    private val robot = TeamRobot()

    override fun init() {
        robot.init(hardwareMap)
        RobotPos.resetAll()
    }

    override fun loop() {
        RobotMovement.goToPosition(358.0 / 2, 358.0 / 2, 1.0, Math.toRadians(90.0), 0.3)
    }
}
