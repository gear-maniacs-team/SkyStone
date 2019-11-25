package org.firstinspires.ftc.teamcode.autonomous

import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.TeamRobot
import org.firstinspires.ftc.teamcode.sensors.Gyro
import org.firstinspires.ftc.teamcode.utils.fastLazy

@TeleOp(name = "TestingIMU")
class TestingIMU : OpMode() {

    private val robot = TeamRobot()
    private val gyro = Gyro()
    private val wheelMotors by fastLazy { robot.wheelsMotors }

    override fun init() {
        robot.init(hardwareMap)
        gyro.init(hardwareMap)
    }

    override fun start() {
        gyro.start()
    }

    override fun loop() {
        telemetry.addData("Angle", gyro.angle)
        telemetry.update()
    }
}