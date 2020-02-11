package net.gearmaniacs.teamcode.autonomous

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import net.gearmaniacs.teamcode.drive.MecanumDrive

@TeleOp(name = "PathFollower")
class PathFollower : LinearOpMode() {

    private val distance = 200.0

    override fun runOpMode() {
        val drive = MecanumDrive(hardwareMap)

        val trajectory = drive.trajectoryBuilder()
            .forward(distance)
            .build()

        waitForStart()

        if (isStopRequested) return

        drive.followTrajectorySync(trajectory)
    }
}