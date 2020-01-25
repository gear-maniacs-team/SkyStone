package net.gearmaniacs.teamcode.teleop

import com.qualcomm.robotcore.eventloop.opmode.TeleOp

@TeleOp(name = "SPARTAN", group = "Good")
class SpartanTeleOp : MainTeleOp() {

    override fun init() {
        robot.init(
            hardwareMap,
            listOf(wheels, intake, lift, foundation, outtake, encoder),
            listOf(encoder)
        )
        super.init()
    }
}
