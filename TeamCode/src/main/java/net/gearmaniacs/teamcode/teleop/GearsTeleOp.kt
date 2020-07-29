package net.gearmaniacs.teamcode.teleop

import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import net.gearmaniacs.teamcode.hardware.sensors.GyroEncoders

@TeleOp(name = "G.E.A.R.S.", group = "Good")
class GearsTeleOp : MainTeleOp() {

    private val gyroEncoders = GyroEncoders()

    override fun init() {
        robot.useBulkRead = false
        robot.init(
            hardwareMap,
            listOf(wheels, intake, lift, foundation, outtake, gyroEncoders),
            listOf(gyroEncoders)
        )
        super.init()
    }
}
