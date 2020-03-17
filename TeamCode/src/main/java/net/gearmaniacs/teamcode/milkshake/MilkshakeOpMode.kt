package net.gearmaniacs.teamcode.milkshake

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DcMotor
import net.gearmaniacs.teamcode.RobotPos
import net.gearmaniacs.teamcode.TeamOpMode
import net.gearmaniacs.teamcode.hardware.motors.Wheels
import net.gearmaniacs.teamcode.hardware.sensors.Encoders
import net.gearmaniacs.teamcode.utils.MathUtils
import kotlin.math.cos
import kotlin.math.sin

@TeleOp(name = "Milkshake")
class MilkshakeOpMode : TeamOpMode() {

    private val wheels = Wheels()
    private val encoder = Encoders()
    private val dashboard: FtcDashboard = FtcDashboard.getInstance()
    private val curves = listOf(
        CurvePoint(0.0, 80.0, 0.15, 0.2, 0.2),
        CurvePoint(80.0, 80.0, 0.15, 0.2, 0.2)
    )
    private val milkshake = Milkshake(curves)

    override fun init() {
        RobotPos.resetAll()
        initRobot(listOf(wheels, encoder), listOf(encoder))

        wheels.setModeAll(DcMotor.RunMode.RUN_USING_ENCODER)
    }

    override fun loop() {
        val robotLocation = RobotLocation(RobotPos.currentX, RobotPos.currentY, RobotPos.targetAngle)
        val result = milkshake.followCurve(robotLocation)
        movement(result.xPower, result.yPower, result.rotationPower)

        with(telemetry) {
            addData("Current X", "%.3f", RobotPos.currentX)
            addData("Current Y", "%.3f", RobotPos.currentY)
            addData("Current Angle", "%.3f", RobotPos.currentAngle)
            addLine()
            addData("Target X", "%.3f", result.xPower)
            addData("Target Y", "%.3f", result.yPower)
            addData("Target Angle", "%.3f", result.rotationPower)
            addLine()
            addData("rightFront", wheels.rightFront.power)
            addData("leftFront", wheels.leftFront.power)
            addData("rightRear", wheels.rightRear.power)
            addData("leftRear", wheels.leftRear.power)
        }

        val packet = TelemetryPacket().apply {
            put("X Position", RobotPos.currentX)
            put("Y Position", RobotPos.currentY)
            put("Rad", MathUtils.angleWrap(RobotPos.currentAngle))
            put("Power X", result.xPower)
            put("Power Y", result.yPower)
            put("Power Rot", result.rotationPower)
        }

        dashboard.sendTelemetryPacket(packet)
    }

    private fun movement(x: Double, y: Double, rot: Double) {
        val currentAngle = RobotPos.currentAngle
        val sinOrientation = sin(currentAngle)
        val cosOrientation = cos(currentAngle)

        val fieldOrientedX = x//x * cosOrientation - y * sinOrientation
        val fieldOrientedY = -y//x * sinOrientation + y * cosOrientation

        with(wheels) {
            leftFront.power = -fieldOrientedX - fieldOrientedY - rot
            leftRear.power = fieldOrientedX - fieldOrientedY - rot
            rightRear.power = -fieldOrientedX - fieldOrientedY + rot
            rightFront.power = fieldOrientedX - fieldOrientedY + rot
        }
    }
}
