package net.gearmaniacs.teamcode.teleop

import com.qualcomm.robotcore.eventloop.opmode.TeleOp

@TeleOp(name = "G.E.A.R.S.", group = "Good")
class GearsTeleOp : MainTeleOp() {

    override fun init() {
        robot.init(
            hardwareMap,
            listOf(wheels, intake, lift, foundation, outtake, gyro),
            listOf(gyro)
        )
        super.init()
    }

    override fun start() {
        super.start()
        gyro.waitForCalibration()
    }
}
