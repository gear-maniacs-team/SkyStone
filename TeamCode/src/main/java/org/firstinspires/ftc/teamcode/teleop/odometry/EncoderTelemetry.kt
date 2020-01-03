package org.firstinspires.ftc.teamcode.teleop.odometry

import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DcMotor
import org.firstinspires.ftc.teamcode.utils.UPSCounter

@TeleOp(name = "EncoderTelemetry")
class EncoderTelemetry : OpMode() {

    private val upsCounter = UPSCounter()
    private lateinit var encoder: DcMotor

    override fun init() {
        encoder = hardwareMap.dcMotor["intake_right"]

        encoder.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
    }

    override fun loop() {
        upsCounter.update(telemetry)
        telemetry.addData("Position", encoder.currentPosition)
        telemetry.update()
    }
}
