package net.gearmaniacs.teamcode.teleop

import com.qualcomm.robotcore.eventloop.opmode.Disabled
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import net.gearmaniacs.teamcode.hardware.sensors.Encoders

@Disabled
@TeleOp(name = "SPARTAN", group = "Good")
class SpartanTeleOp : MainTeleOp() {

    private val encoder = Encoders()

    override fun init() {
        robot.init(
            hardwareMap,
            listOf(wheels, intake, lift, foundation, outtake, encoder),
            listOf(encoder)
        )
        super.init()
    }
}
