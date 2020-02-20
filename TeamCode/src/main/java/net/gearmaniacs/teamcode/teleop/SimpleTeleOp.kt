package net.gearmaniacs.teamcode.teleop

import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import net.gearmaniacs.teamcode.hardware.sensors.Gyro
import net.gearmaniacs.teamcode.hardware.sensors.GyroEncoders

@TeleOp(name = "A Simple TeleOp", group = "Good")
class SimpleTeleOp : MainTeleOp() {

    override fun init() {
        robot.useBulkRead = false
        robot.init(
            hardwareMap,
            listOf(wheels, intake, lift, foundation, outtake)
        )
        super.init()
    }
}
