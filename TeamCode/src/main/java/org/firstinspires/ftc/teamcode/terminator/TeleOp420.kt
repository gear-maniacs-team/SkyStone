package org.firstinspires.ftc.teamcode.terminator

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import net.gearmaniacs.teamcode.RobotPos
import net.gearmaniacs.teamcode.TeamRobot
import net.gearmaniacs.teamcode.hardware.sensors.Encoders

@TeleOp(name = "ElectricBoogaloo", group = "Boogaloo")
class TeleOp420 : LinearOpMode() {

    private val robot = TeamRobot()
    private val wheels = Wheels()
    private val encoder = Encoders()
    private var precisionValue = 1.0

    override fun runOpMode() {
        RobotPos.resetAll()
        robot.init(hardwareMap, listOf(wheels, encoder), listOf(encoder))
        robot.start()
        waitForStart()

        wheels.frontLeft.power = 0.0
        wheels.frontRight.power = 0.0
        wheels.backLeft.power = 0.0
        wheels.backRight.power = 0.0

        while (opModeIsActive()) {
            val x1 = gamepad1.left_stick_x.toDouble()
            val y1 = -gamepad1.left_stick_y.toDouble()
            val x2 = gamepad1.right_stick_x.toDouble()

            wheels.frontLeft.power = (y1 + x1 + x2) / precisionValue
            wheels.frontRight.power = (y1 - x1 - x2) / precisionValue
            wheels.backLeft.power = (y1 - x1 + x2) / precisionValue
            wheels.backRight.power = (y1 + x1 - x2) / precisionValue

            precisionValue = if (gamepad1.right_trigger >= 0.5) 3.0 else 1.0

            if (gamepad1.b)
                break
        }

        robot.stop()
    }
}
