package org.firstinspires.ftc.teamcode.terminator

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import net.gearmaniacs.teamcode.RobotPos
import net.gearmaniacs.teamcode.TeamRobot
import net.gearmaniacs.teamcode.hardware.sensors.Encoders

//adb connect 192.168.43.1
@Autonomous(name = "Boogaloo", group = "Boogaloo")
class FoundationRed : LinearOpMode() {

    private val YPID = PIDController(0.02,0.0000005,4.2)
    private val APID = PIDController(0.02,0.0000005,4.2)
    private val XPID = PIDController(0.02,0.0000005,4.2)
    private val robot = TeamRobot()
    private val wheels = Wheels()
    private val encoders = Encoders()

    override fun runOpMode() {
        RobotPos.resetAll()
        robot.init(hardwareMap, listOf(encoders), listOf(encoders))
        wheels.init(hardwareMap)
        robot.start()
        waitForStart()
        YPID.setPoint = 60.0
        APID.setPoint = 0.0
        XPID.setPoint = 0.0
        YPID.setOutputRange(-1.0, 1.0)
        APID.setOutputRange(-1.0, 1.0)
        XPID.setOutputRange(-1.0, 1.0)

        while (opModeIsActive()){
            val yResult = YPID.performPID(RobotPos.currentY)
            val aResult = APID.performPID(Math.toDegrees(RobotPos.currentAngle))
            val xResult = XPID.performPID(RobotPos.currentX)

            wheels.frontLeft.power = yResult + aResult + xResult
            wheels.frontRight.power = yResult - aResult - xResult
            wheels.backLeft.power = yResult + aResult - xResult
            wheels.backRight.power = yResult - aResult + xResult

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
