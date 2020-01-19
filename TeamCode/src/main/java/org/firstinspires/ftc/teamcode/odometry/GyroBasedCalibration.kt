package org.firstinspires.ftc.teamcode.odometry

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DcMotor
import org.firstinspires.ftc.teamcode.RobotPos
import org.firstinspires.ftc.teamcode.TeamRobot
import org.firstinspires.ftc.teamcode.motors.Intake
import org.firstinspires.ftc.teamcode.motors.Wheels
import org.firstinspires.ftc.teamcode.sensors.Encoder
import org.firstinspires.ftc.teamcode.sensors.Gyro

@TeleOp(name = "Actually calibrate")
class GyroBasedCalibration : LinearOpMode() {

    private val robot = TeamRobot()
    private val wheels = Wheels()
    private val encoder = Encoder()
    private val gyro = Gyro()
    private val distance = 0.0
    private lateinit var leftEnc: DcMotor
    private lateinit var rightEnc: DcMotor

    override fun runOpMode() {
        leftEnc = hardwareMap.dcMotor["TL"]
        rightEnc = hardwareMap.dcMotor["BR"]
        robot.init(hardwareMap, listOf(wheels,gyro), listOf(gyro))
        waitForStart()
        robot.start()

        wheels.leftBack.power = 0.5
        wheels.leftFront.power = 0.5
        wheels.rightBack.power = 0.5
        wheels.rightFront.power = 0.5

        Thread.sleep(1000)

        telemetry.addData("Result", (leftEnc.currentPosition - rightEnc.currentPosition)/RobotPos.currentAngle)
        telemetry.update()

        robot.stop()
    }

}