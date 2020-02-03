package net.gearmaniacs.teamcode.autonomous

import com.acmerobotics.roadrunner.control.PIDCoefficients
import com.acmerobotics.roadrunner.control.PIDFController
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.hardware.DcMotor
import net.gearmaniacs.teamcode.RobotPos
import net.gearmaniacs.teamcode.TeamRobot
import net.gearmaniacs.teamcode.hardware.motors.Wheels
import net.gearmaniacs.teamcode.hardware.sensors.Encoders

@Autonomous(name = "Helper")
class ProfileSpeedHelper: OpMode() {

    private val robot = TeamRobot()
    private val encoder = Encoders()
    private val wheels = Wheels()

    override fun init() {
        robot.useBulkRead = false
        robot.init(
            hardwareMap,
            listOf(wheels, encoder),
            listOf(encoder)
        )

        wheels.setModeAll(DcMotor.RunMode.STOP_AND_RESET_ENCODER)
        wheels.setModeAll(DcMotor.RunMode.RUN_USING_ENCODER)
    }

    override fun start() {
        robot.start()
        RobotPos.resetAll()
    }

    override fun loop() {
        wheels.setPowerAll(1.0)

    }

    override fun stop() {

    }

}