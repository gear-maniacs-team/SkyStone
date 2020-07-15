package org.firstinspires.ftc.teamcode.terminator

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import net.gearmaniacs.teamcode.RobotPos
import net.gearmaniacs.teamcode.TeamRobot
import net.gearmaniacs.teamcode.hardware.sensors.Encoders

@TeleOp(name = "OdemetryBoomBoom")
class OdemetryBoomBoom : LinearOpMode() {

    private val robot = TeamRobot()
    private val wheels = Wheels()
    private val encoders = Encoders()

    override fun runOpMode() {
        RobotPos.resetAll()
        robot.init(hardwareMap, listOf(encoders), listOf(encoders))
        wheels.init(hardwareMap)
        robot.start()
        waitForStart()

        while (opModeIsActive()){
            val packet = TelemetryPacket().apply {
                put("XPos", RobotPos.currentX)
                put("YPos", RobotPos.currentY)
                put("Angle", Math.toDegrees(RobotPos.currentAngle))
            }
            FtcDashboard.getInstance().sendTelemetryPacket(packet)
        }

        robot.stop()
    }
}
