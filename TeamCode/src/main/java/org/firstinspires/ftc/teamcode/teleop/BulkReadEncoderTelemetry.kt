package org.firstinspires.ftc.teamcode.teleop

import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DcMotor
import org.firstinspires.ftc.teamcode.TeamRobot
import org.firstinspires.ftc.teamcode.utils.PerformanceProfiler
import org.openftc.revextensions2.ExpansionHubMotor

@TeleOp(name = "BulkEncoderTelemetry")
class BulkReadEncoderTelemetry : OpMode() {

    private val robot = TeamRobot()
    private val upsCounter = PerformanceProfiler()
    private lateinit var encoder: ExpansionHubMotor

    override fun init() {
        robot.init(hardwareMap)

        encoder = hardwareMap.dcMotor["intake_right"] as ExpansionHubMotor

        encoder.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
    }

    override fun loop() {
        upsCounter.update(telemetry)

        telemetry.addData("Position", robot.bulkInputData2.getMotorCurrentPosition(encoder))
        telemetry.update()
    }
}
