package org.firstinspires.ftc.teamcode.terminator

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import net.gearmaniacs.teamcode.hardware.servos.FoundationServos

//adb connect 192.168.43.1
@Autonomous(name = "Foundation_Red", group = "Boogaloo")
class FoundationRed : LinearOpMode() {

    private val wheels = Wheels()
    private val foundation = FoundationServos()

    override fun runOpMode() {
        wheels.init(hardwareMap)
        foundation.init(hardwareMap)

        waitForStart()
        wheels.waitForMotors()

        wheels.goBackCm(65.0, 0.3)
        foundation.attach()

        wheels.goForwardCm(30.0,0.3)
        wheels.rotateRight(60.0,0.3)
        wheels.goLeftCm(60.0,0.3)
        foundation.detach()
        wheels.goForwardCm(70.0,0.5)

        //while(OpModeIsActive)
    }
}

@Autonomous(name = "Foundation_Blue", group = "Boogaloo")
class FoundationBlue : LinearOpMode() {

    private val wheels = Wheels()
    private val foundation = FoundationServos()

    override fun runOpMode() {
        wheels.init(hardwareMap)
        foundation.init(hardwareMap)

        waitForStart()
        wheels.waitForMotors()

        wheels.goRightCm(30.0,0.3)
        wheels.goBackCm(120.0, 0.3)
        wheels.waitForMotors()
        foundation.attach()

        wheels.goForwardCm(60.0, 0.3)
        wheels.rotateLeft(100.0, 0.3)
        wheels.goRightCm(100.0, 0.3)
        wheels.waitForMotors()
        foundation.detach()
        wheels.goForwardCm(70.0, 0.5)

    }

}
