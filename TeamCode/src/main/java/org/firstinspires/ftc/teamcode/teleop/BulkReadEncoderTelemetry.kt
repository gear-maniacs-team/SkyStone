package org.firstinspires.ftc.teamcode.teleop

import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DcMotor
import org.firstinspires.ftc.teamcode.utils.UPSCounter
import org.openftc.revextensions2.ExpansionHubEx
import org.openftc.revextensions2.ExpansionHubMotor

@TeleOp(name = "BulkEncoderTelemetry")
class BulkReadEncoderTelemetry : OpMode() {

    private val upsCounter = UPSCounter()
    private lateinit var expansionHub: ExpansionHubEx
    private lateinit var encoder: ExpansionHubMotor

    override fun init() {
        expansionHub = hardwareMap.get(ExpansionHubEx::class.java, "Expansion Hub 2")

        encoder = hardwareMap.dcMotor["intake_right"] as ExpansionHubMotor

        encoder.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
    }

    override fun loop() {
        upsCounter.update(telemetry)

        val bulkData = expansionHub.bulkInputData

        telemetry.addData("Position", bulkData.getMotorCurrentPosition(encoder))
        telemetry.update()
    }
}
