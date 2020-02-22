package net.gearmaniacs.teamcode.teleop

import com.qualcomm.robotcore.eventloop.opmode.Disabled
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.Servo
import net.gearmaniacs.teamcode.utils.extensions.getDevice

@Disabled
@TeleOp(name = "ServoProgrammer")
open class ServoProgrammer : OpMode() {

    private lateinit var servo: Servo
    private var position = 0.0

    override fun init() {
        servo = hardwareMap.getDevice("")
    }

    override fun loop() {
        val change = when {
            gamepad1.dpad_up -> 0.05
            gamepad1.dpad_up -> -0.05
            else -> 0.0
        }

        if (change != 0.0)
            Thread.sleep(400)

        position += change

        servo.position = position
    }
}
