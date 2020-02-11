package net.gearmaniacs.teamcode.teleop.odometry

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.canvas.Canvas
import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.acmerobotics.roadrunner.geometry.Pose2d
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import net.gearmaniacs.teamcode.RobotPos
import net.gearmaniacs.teamcode.TeamOpMode
import net.gearmaniacs.teamcode.drive.DashboardUtil
import net.gearmaniacs.teamcode.hardware.sensors.Encoders
import net.gearmaniacs.teamcode.utils.MathUtils.angleWrap
import net.gearmaniacs.teamcode.utils.PerformanceProfiler
import net.gearmaniacs.teamcode.utils.extensions.CM_TO_INCH
import net.gearmaniacs.teamcode.utils.extensions.getCurrentPosition
import kotlin.math.pow

@TeleOp(name = "Odometry Test", group = "Odometry")
class OdometryTest : TeamOpMode() {

    private val encoder = Encoders()
    private val performanceProfiler = PerformanceProfiler()
    private val dashboard: FtcDashboard = FtcDashboard.getInstance()

    private val maxTrajectorySize = 2.0.pow(20.0).toInt()
    private val xTrajectory = FloatArray(maxTrajectorySize)
    private val yTrajectory = FloatArray(maxTrajectorySize)
    private var nextIndexTrajectory = 1

    override fun init() {
        FtcDashboard.getInstance().telemetryTransmissionInterval = 250
        initRobot(listOf(encoder), listOf(encoder))
        RobotPos.resetAll()
    }

    override fun loop() {
        performanceProfiler.update(telemetry)

        val leftPos = -encoder.left.getCurrentPosition(robot.bulkData2)
        val rightPos = encoder.right.getCurrentPosition(robot.bulkData1)
        val backPos = encoder.back.getCurrentPosition(robot.bulkData2)

        with(telemetry) {
            addData("X Position", "%.3f", RobotPos.currentX)
            addData("Y Position", "%.3f", RobotPos.currentY)
            addData("Radians", "%.3f", angleWrap(RobotPos.currentAngle))
            addLine()
            addData("Left encoder", leftPos)
            addData("Right encoder", rightPos)
            addData("Back encoder", backPos)
        }

        val packet = TelemetryPacket().apply {
            put("X Position", RobotPos.currentX)
            put("Y Position", RobotPos.currentY)
            put("Radians", angleWrap(RobotPos.currentAngle))
            put("Left encoder", leftPos)
            put("Right encoder", rightPos)
            put("Back encoder", backPos)
        }

        val x = (RobotPos.currentY * CM_TO_INCH).toFloat()
        val y = (-RobotPos.currentX * CM_TO_INCH).toFloat()

        val fieldOverlay: Canvas = packet.fieldOverlay()
        fieldOverlay.setStroke("#3F51B5")
        fieldOverlay.fillCircle(x.toDouble(), y.toDouble(), 4.0)

        if (x != xTrajectory.last() || y != yTrajectory.last()) {
            xTrajectory[nextIndexTrajectory]
            yTrajectory[nextIndexTrajectory]
            ++nextIndexTrajectory
        }

        DashboardUtil.drawRobot(fieldOverlay, Pose2d(x.toDouble(), y.toDouble(), RobotPos.currentAngle))
        drawSampledPath(fieldOverlay)
        dashboard.sendTelemetryPacket(packet)

        if (nextIndexTrajectory >= maxTrajectorySize)
            nextIndexTrajectory = 0
    }

    private fun FloatArray.toDoubleArray(): DoubleArray {
        val result = DoubleArray(nextIndexTrajectory)
        for (i in 0 until nextIndexTrajectory)
            result[i] = this[i].toDouble()
        return result
    }

    private fun drawSampledPath(canvas: Canvas) {
        val xPoints = xTrajectory.toDoubleArray()
        val yPoints = yTrajectory.toDoubleArray()

        canvas.strokePolyline(xPoints, yPoints)
    }
}
