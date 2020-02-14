package net.gearmaniacs.teamcode.autonomous.demo

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.Disabled
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import net.gearmaniacs.teamcode.TeamRobot
import net.gearmaniacs.teamcode.hardware.motors.Wheels

abstract class ParkingAuto : LinearOpMode() {

    private val robot = TeamRobot()
    private val wheels = Wheels()

    final override fun runOpMode() {
        robot.init(hardwareMap, listOf(wheels))

        while (!isStarted) {
            telemetry.addData("Status", "Waiting for start")
            telemetry.update()
        }

        robot.start()

        onRun()

        robot.stop()
    }

    abstract fun onRun()

    protected fun moveForward(power: Double, time: Long) {
        with(wheels) {
            rightFront.power = power
            leftFront.power = -power
            rightRear.power = power
            leftRear.power = -power
        }

        waitTime(time)
    }

    protected fun strafe(power: Double, time: Long) {
        with(wheels) {
            rightFront.power = -power
            leftFront.power = -power
            rightRear.power = power
            leftRear.power = power
        }

        waitTime(time)
    }

    private fun waitTime(time: Long) {
        if (!opModeIsActive()) return
        val targetTime = System.currentTimeMillis() + time

        do {
            Thread.sleep(10)
        } while (opModeIsActive() && targetTime > System.currentTimeMillis())
    }
}

@Disabled
@Autonomous(name = "Blue-Parking", group = "Parking")
class BlueAuto : ParkingAuto() {

    override fun onRun() {
        strafe(-0.4, 1600L)
        moveForward(-0.4, 1400L)
        strafe(-0.3, 1500L)
    }
}

@Disabled
@Autonomous(name = "Red-Parking", group = "Parking")
class RedAuto : ParkingAuto() {

    override fun onRun() {
        strafe(0.4, 1600L)
        moveForward(-0.4, 1400L)
        strafe(0.25, 1500L)
    }
}

@Disabled
@Autonomous(name = "Simple-Parking", group = "Parking")
class SimpleAuto : ParkingAuto() {

    override fun onRun() {
        moveForward(0.4, 1200L)
    }
}
