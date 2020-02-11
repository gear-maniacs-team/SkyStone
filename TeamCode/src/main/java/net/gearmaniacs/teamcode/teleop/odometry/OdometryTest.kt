package net.gearmaniacs.teamcode.teleop.odometry

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.canvas.Canvas
import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import net.gearmaniacs.teamcode.RobotPos
import net.gearmaniacs.teamcode.TeamOpMode
import net.gearmaniacs.teamcode.hardware.sensors.Encoders
import net.gearmaniacs.teamcode.utils.MathUtils.angleWrap
import net.gearmaniacs.teamcode.utils.PerformanceProfiler
import net.gearmaniacs.teamcode.utils.extensions.CM_TO_INCH
import net.gearmaniacs.teamcode.utils.extensions.getCurrentPosition
import kotlin.math.pow

@TeleOp(name = "Odometry Test", group = "Odometry")
class OdometryTest : TeamOpMode() {

    private val maxTrajectorySize = 2.0.pow(16.0)
    private val xTrajectory = ArrayList<Double>(maxTrajectorySize.toInt())
    private val yTrajectory = ArrayList<Double>(maxTrajectorySize.toInt())
    private val encoder = Encoders()
    private val performanceProfiler = PerformanceProfiler()
    private val dashboard: FtcDashboard = FtcDashboard.getInstance()

    override fun init() {
        robot.init(hardwareMap, listOf(encoder), listOf(encoder))
        RobotPos.resetAll()
    }

    override fun loop() {
        performanceProfiler.update(telemetry)

        telemetry.addData("X Position", "%.3f", RobotPos.currentX)
        telemetry.addData("Y Position", "%.3f", RobotPos.currentY)
        telemetry.addData("Radians", "%.3f", angleWrap(RobotPos.currentAngle))
        telemetry.addData("--", "--")

        val leftPos = -encoder.left.getCurrentPosition(robot.bulkData2)
        val rightPos = encoder.right.getCurrentPosition(robot.bulkData1)
        val backPos = encoder.back.getCurrentPosition(robot.bulkData2)

        telemetry.addData("Left encoder", leftPos)
        telemetry.addData("Right encoder", rightPos)
        telemetry.addData("Back encoder", backPos)

        val packet = TelemetryPacket().apply {
            put("X Position", RobotPos.currentX)
            put("Y Position", RobotPos.currentY)
            put("Radians", angleWrap(RobotPos.currentAngle))
            put("Left encoder", leftPos)
            put("Right encoder", rightPos)
            put("Back encoder", backPos)
        }

        val x = RobotPos.currentY * CM_TO_INCH
        val y = -RobotPos.currentX * CM_TO_INCH
        val fieldOverlay: Canvas = packet.fieldOverlay()
        fieldOverlay.setStroke("#3F51B5")
        fieldOverlay.fillCircle(x, y, 4.0)

        if (x != xTrajectory.lastOrNull() || y == yTrajectory.lastOrNull()) {
            xTrajectory.add(x)
            yTrajectory.add(y)
        }

        drawSampledPath(fieldOverlay)
        dashboard.sendTelemetryPacket(packet)
    }

    private fun drawSampledPath(canvas: Canvas) {
        val xPoints = xTrajectory.toDoubleArray()
        val yPoints = yTrajectory.toDoubleArray()

        canvas.strokePolyline(xPoints, yPoints)
    }
}
