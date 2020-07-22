package org.firstinspires.ftc.teamcode.terminator

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import net.gearmaniacs.teamcode.RobotPos
import net.gearmaniacs.teamcode.TeamRobot
import net.gearmaniacs.teamcode.hardware.sensors.Encoders
import net.gearmaniacs.teamcode.utils.RobotClock

//adb connect 192.168.43.1
@Autonomous(name = "Boogaloo", group = "Boogaloo")
class FoundationRed : LinearOpMode() {

    private var pidSystem = PIDSystem()
    private val robot = TeamRobot()
    private val wheels = Wheels()
    private val encoders = Encoders()

    override fun runOpMode() {
        RobotPos.resetAll()
        robot.init(hardwareMap, listOf(encoders), listOf(encoders))
        wheels.init(hardwareMap)
        robot.start()
        waitForStart()

        pidSystem.addPoints(PathPoint(45.0, 45.0, 0.0), PathPoint(0.0,0.0,0.0))
        pidSystem.init()

        while (opModeIsActive()){

            pidSystem.run(robot, wheels)

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
