package org.firstinspires.ftc.teamcode.terminator

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode

//adb connect 192.168.43.1

@Autonomous(name = "MoveRight", group = "Boogaloo")
class WheelsMovement : LinearOpMode() {

    val wheels = Wheels()

    override fun runOpMode() {
        wheels.init(hardwareMap)
        waitForStart()

        wheels.goRightCm(5.0, 0.2)
        wheels.waitForMotors()

        //while(OpModeIsActive)
    }
}
