package net.gearmaniacs.teamcode.drive

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
import org.firstinspires.ftc.robotcore.external.Telemetry
import java.util.ArrayList

abstract class MecanumDriveBase(
    private val telemetry: Telemetry
) : MecanumDrive(
    Drive.kV,
    Drive.kA,
    Drive.kStatic,
    Drive.TRACK_WIDTH
) {

    enum class Mode {
        IDLE, TURN, FOLLOW_TRAJECTORY
    }

    private val clock: NanoClock = NanoClock.system()
    private var mode = Mode.IDLE
    private val turnController = PIDFController(HEADING_PID).apply {
        setInputBounds(0.0, 2 * Math.PI)
    }
    private var turnProfile: MotionProfile? = null
    private var turnStart = 0.0
    private val constraints = MecanumConstraints(Drive.BASE_CONSTRAINTS, Drive.TRACK_WIDTH)
    private val follower = HolonomicPIDVAFollower(
        TRANSLATIONAL_PID,
        TRANSLATIONAL_PID,
        HEADING_PID
    )
    private var lastWheelPositions: List<Double>? = null
    private var lastTimestamp = 0.0

    fun trajectoryBuilder(): TrajectoryBuilder {
        return TrajectoryBuilder(poseEstimate, constraints)
    }

    fun turn(angle: Double) {
        val heading = poseEstimate.heading
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

    fun followTrajectory(trajectory: Trajectory?) {
        follower.followTrajectory(trajectory!!)
        mode = Mode.FOLLOW_TRAJECTORY
    }

    fun followTrajectorySync(trajectory: Trajectory?) {
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
        telemetry.addData("mode", mode)
        telemetry.addData("x", currentPose.x)
        telemetry.addData("y", currentPose.y)
        telemetry.addData("heading", currentPose.heading)
        telemetry.addData("xError", lastError.x)
        telemetry.addData("yError", lastError.y)
        telemetry.addData("headingError", lastError.heading)
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
                TODO("Add Trajectory support")
            }
        }
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
        var TRANSLATIONAL_PID = PIDCoefficients(0.0, 0.0, 0.0)
        var HEADING_PID = PIDCoefficients(0.0, 0.0, 0.0)
    }
}
