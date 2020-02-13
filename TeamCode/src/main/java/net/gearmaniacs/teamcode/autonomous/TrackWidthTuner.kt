package net.gearmaniacs.teamcode.autonomous

import com.acmerobotics.roadrunner.util.Angle.norm
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.util.MovingStatistics
import net.gearmaniacs.teamcode.RobotPos
import net.gearmaniacs.teamcode.TeamRobot
import net.gearmaniacs.teamcode.drive.Drive
import net.gearmaniacs.teamcode.drive.MecanumDriveBase
import net.gearmaniacs.teamcode.drive.MecanumDrive
import net.gearmaniacs.teamcode.hardware.sensors.Encoders
import org.firstinspires.ftc.robotcore.internal.system.Misc
import kotlin.math.sqrt

@Autonomous(group = "TrackWidthTuner")
class TrackWidthTuner : LinearOpMode() {

    override fun runOpMode() {
        val drive: MecanumDriveBase = MecanumDrive(hardwareMap)
        val robot = TeamRobot()
        val encoders = Encoders()

        robot.init(hardwareMap, listOf(encoders), listOf(encoders))
        telemetry.addLine("Press play to begin the track width tuner routine")
        telemetry.addLine("Make sure your robot has enough clearance to turn smoothly")
        telemetry.update()
        waitForStart()
        robot.start()
        if (isStopRequested) return

        telemetry.clearAll()
        telemetry.addLine("Running...")
        telemetry.update()
        val trackWidthStats = MovingStatistics(NUM_TRIALS)

        for (i in 0 until NUM_TRIALS) {
            RobotPos.resetAll()
            // it is important to handle heading wraparounds
            var headingAccumulator = 0.0
            var lastHeading = 0.0
            drive.turn(Math.toRadians(ANGLE))
            while (!isStopRequested && drive.isBusy) {
                val heading = RobotPos.currentAngle
                headingAccumulator += norm(heading - lastHeading)
                lastHeading = heading
                drive.update()
            }
            val trackWidth: Double = Drive.TRACK_WIDTH * Math.toRadians(ANGLE) / headingAccumulator
            trackWidthStats.add(trackWidth)
            sleep(DELAY.toLong())
        }

        telemetry.clearAll()
        telemetry.addLine("Tuning complete")
        telemetry.addLine(
            Misc.formatInvariant(
                "Effective track width = %.2f (SE = %.3f)",
                trackWidthStats.mean,
                trackWidthStats.standardDeviation / sqrt(NUM_TRIALS.toDouble())
            )
        )
        telemetry.update()

        while (!isStopRequested) {
            idle()
        }
        robot.stop()
    }

    companion object {
        private const val ANGLE = 180.0 // deg
        private const val NUM_TRIALS = 5
        private const val DELAY = 1000 // ms
    }
}