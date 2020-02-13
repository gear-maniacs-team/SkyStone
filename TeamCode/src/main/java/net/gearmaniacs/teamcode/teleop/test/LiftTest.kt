package net.gearmaniacs.teamcode.teleop.test

import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import net.gearmaniacs.teamcode.TeamOpMode
import net.gearmaniacs.teamcode.hardware.motors.Lift

@TeleOp(name = "LiftTest", group = "Test")
class LiftTest : TeamOpMode() {

    private val maxTarget = 1500
    private val liftPower = 1.0
    private val lift = Lift()
    private var liftTargetPosition = 0

    override fun init() {
        robot.useBulkRead = false
        robot.init(hardwareMap, listOf(lift))

        lift.setPowerAll(liftPower)
    }

    override fun loop() {
        val posChange = when {
            gamepad2.dpad_down -> -25
            gamepad2.dpad_up -> 25
            else -> 0
        }

        liftTargetPosition += posChange

        if (liftTargetPosition > maxTarget)
            liftTargetPosition = maxTarget
        if (liftTargetPosition < 0)
            liftTargetPosition = 0

        if (posChange != 0)
            Thread.sleep(200)

        lift.setTargetPositionAll(liftTargetPosition)

        telemetry.addData("Current Position Left", lift.left.currentPosition)
        telemetry.addData("Current Position Right", lift.right.currentPosition)
        telemetry.addData("Target Position", liftTargetPosition)
    }
}
