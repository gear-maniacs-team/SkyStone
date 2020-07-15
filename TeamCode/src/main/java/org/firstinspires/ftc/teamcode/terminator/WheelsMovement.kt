package org.firstinspires.ftc.teamcode.terminator

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode

//adb connect 192.168.43.1
@Autonomous(name = "Boogaloo", group = "Boogaloo")
class FoundationRed : LinearOpMode() {

    private val wheels = Wheels()

    override fun runOpMode() {
        wheels.init(hardwareMap)
        waitForStart()

//        wheels.goRightForward(50.0,0.3)
//        wheels.waitForMotors()
//        Thread.sleep(1000)

        wheels.goRightCm(50.0,0.3)
        wheels.waitForMotors()
        Thread.sleep(1000)
//
//        wheels.goForwardCm(100.0,0.3)
//        wheels.waitForMotors()
//        Thread.sleep(1000)
//
//        wheels.goLeftCm(50.0,0.3)
//        wheels.waitForMotors()
//        Thread.sleep(1000)
//
//        wheels.goBackCm(50.0,0.3)
//        wheels.waitForMotors()
//        Thread.sleep(1000)

        //while(OpModeIsActive)
    }
}
