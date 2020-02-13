package net.gearmaniacs.teamcode.drive

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.canvas.Canvas
import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.acmerobotics.roadrunner.control.PIDCoefficients
import com.acmerobotics.roadrunner.control.PIDFController
import com.acmerobotics.roadrunner.drive.DriveSignal
import com.acmerobotics.roadrunner.drive.MecanumDrive
import com.acmerobotics.roadrunner.followers.HolonomicPIDVAFollower
import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.profile.MotionProfile
import com.acmerobotics.roadrunner.profile.MotionProfileGenerator
import com.acmerobotics.roadrunner.profile.MotionState
import com.acmerobotics.roadrunner.trajectory.Trajectory
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumConstraints
import com.acmerobotics.roadrunner.util.NanoClock
import com.qualcomm.robotcore.hardware.DcMotor.RunMode
import net.gearmaniacs.teamcode.utils.RobotClock
import net.gearmaniacs.teamcode.utils.extensions.CM_TO_INCH
import java.util.*

abstract class MecanumDriveBase : MecanumDrive(
    Drive.kV,
    Drive.kA,
    Drive.kStatic,
    Drive.TRACK_WIDTH
) {

    enum class Mode {
        IDLE, TURN, FOLLOW_TRAJECTORY
    }

    private val dashboard: FtcDashboard = FtcDashboard.getInstance()
    private val clock: NanoClock = NanoClock.system()
    private var mode = Mode.IDLE
    private val turnController = PIDFController(HEADING_PID).apply {
        setInputBounds(-Math.PI, Math.PI)
    }
    private var turnProfile: MotionProfile? = null
    private var turnStart = 0.0
    private val constraints = MecanumConstraints(Drive.BASE_CONSTRAINTS, Drive.TRACK_WIDTH)
    private val follower = HolonomicPIDVAFollower(
        TRANSLATIONAL_PID,
        TRANSLATIONAL_PID,
        HEADING_PID,
        clock = RobotClock
    )
    private var lastWheelPositions: List<Double>? = null
    private var lastTimestamp = 0.0

    fun trajectoryBuilder(): TrajectoryBuilder {
        return TrajectoryBuilder(poseEstimate, constraints)
    }

    fun turn(angle: Double) {
        val heading = rawExternalHeading //poseEstimate.heading
        turnProfile = MotionProfileGenerator.generateSimpleMotionProfile(
            MotionState(heading, 0.0, 0.0, 0.0),
            MotionState(heading + angle, 0.0, 0.0, 0.0),
            constraints.maxAngVel,
            constraints.maxAngAccel,
            constraints.maxAngJerk
        )
        turnStart = clock.seconds()
        mode = Mode.TURN
    }

    fun turnSync(angle: Double) {
        turn(angle)
        waitForIdle()
    }

    fun followTrajectory(trajectory: Trajectory) {
        follower.followTrajectory(trajectory)
        mode = Mode.FOLLOW_TRAJECTORY
    }

    fun followTrajectorySync(trajectory: Trajectory) {
        followTrajectory(trajectory)
        waitForIdle()
    }

    val lastError: Pose2d
        get() {
            return when (mode) {
                Mode.FOLLOW_TRAJECTORY -> follower.lastError
                Mode.TURN -> Pose2d(0.0, 0.0, turnController.lastError)
                Mode.IDLE -> Pose2d()
            }
        }

    fun update() {
        updatePoseEstimate()

        val currentPose = poseEstimate
        val lastError = lastError
        val packet = TelemetryPacket()
        val fieldOverlay: Canvas = packet.fieldOverlay()

        packet.put("mode", mode)
        packet.put("x", currentPose.x)
        packet.put("y", currentPose.y)
        packet.put("heading", currentPose.heading)
        packet.put("xError", lastError.x)
        packet.put("yError", lastError.y)
        packet.put("headingError", lastError.heading)
        when (mode) {
            Mode.IDLE -> {
            }
            Mode.TURN -> {
                val t = clock.seconds() - turnStart
                val targetState = turnProfile!![t]
                turnController.targetPosition = targetState.x
                val targetOmega = targetState.v
                val targetAlpha = targetState.a
                val correction = turnController.update(currentPose.heading, targetOmega)
                setDriveSignal(
                    DriveSignal(
                        Pose2d(0.0, 0.0, targetOmega + correction),
                        Pose2d(0.0, 0.0, targetAlpha)
                    )
                )
                if (t >= turnProfile!!.duration()) {
                    mode = Mode.IDLE
                    setDriveSignal(DriveSignal())
                }
            }
            Mode.FOLLOW_TRAJECTORY -> {
                setDriveSignal(follower.update(currentPose))

                val trajectory = follower.trajectory

                fieldOverlay.setStrokeWidth(1)
                fieldOverlay.setStroke("4CAF50")
                DashboardUtil.drawSampledPath(fieldOverlay, trajectory.path)

                fieldOverlay.setStroke("#F44336")
                val t = follower.elapsedTime()
                DashboardUtil.drawRobot(fieldOverlay, trajectory[t])

                fieldOverlay.setStroke("#3F51B5")
                fieldOverlay.fillCircle(currentPose.x * CM_TO_INCH, currentPose.y * CM_TO_INCH, 3.0)

                if (!follower.isFollowing()) {
                    mode = Mode.IDLE
                    setDriveSignal(DriveSignal())
                }
            }
        }

        dashboard.sendTelemetryPacket(packet)
    }

    fun waitForIdle() {
        while (!Thread.currentThread().isInterrupted && isBusy) {
            update()
        }
    }

    val isBusy: Boolean
        get() = mode != Mode.IDLE

    open val wheelVelocities: List<Double>
        get() {
            val positions = getWheelPositions()
            val currentTimestamp = clock.seconds()
            val velocities: MutableList<Double> =
                ArrayList(positions.size)
            if (lastWheelPositions != null) {
                val dt = currentTimestamp - lastTimestamp
                for (i in positions.indices) {
                    velocities.add((positions[i] - lastWheelPositions!![i]) / dt)
                }
            } else {
                for (i in positions.indices) {
                    velocities.add(0.0)
                }
            }
            lastTimestamp = currentTimestamp
            lastWheelPositions = positions
            return velocities
        }

    abstract fun getPIDCoefficients(runMode: RunMode?): PIDCoefficients?

    abstract fun setPIDCoefficients(
        runMode: RunMode?,
        coefficients: PIDCoefficients
    )

    companion object {
        val TRANSLATIONAL_PID = PIDCoefficients(0.1, 0.0, 0.0)
        val HEADING_PID = PIDCoefficients(0.1, 0.0, 0.0)
    }
}
