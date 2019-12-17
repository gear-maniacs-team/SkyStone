package org.firstinspires.ftc.teamcode.autonomous.demo

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import org.firstinspires.ftc.teamcode.TeamRobot
import org.firstinspires.ftc.teamcode.motors.Wheels

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

    protected fun moveForward(velocity: Double, time: Long) {
        with(wheels) {
            val frontVelocity = getFrontVelocity(velocity)
            val backVelocity = getBackVelocity(velocity)

            rightFront.velocity = frontVelocity
            leftFront.velocity = -frontVelocity
            rightBack.velocity = backVelocity
            leftBack.velocity = -backVelocity
        }

        waitTime(time)
    }

    protected fun strafe(velocity: Double, time: Long) {
        with(wheels) {
            val frontVelocity = getFrontVelocity(velocity)
            val backVelocity = getBackVelocity(velocity)

            rightFront.velocity = -frontVelocity
            leftFront.velocity = -frontVelocity
            rightBack.velocity = backVelocity
            leftBack.velocity = backVelocity
        }

        waitTime(time)
    }

    private fun waitTime(time: Long) {
        val targetTime = System.currentTimeMillis() + time

        do {
            Thread.sleep(10)
        } while (targetTime > System.currentTimeMillis())
    }

    private fun getFrontVelocity(power: Double) =
        Wheels.rpmToTps(MAX_RPM * power, FRONT_ENCODER_COUNT)

    private fun getBackVelocity(power: Double) =
        Wheels.rpmToTps(MAX_RPM * power, BACK_ENCODER_COUNT)

    private companion object {
        private const val MAX_RPM = 223.0
        private const val FRONT_ENCODER_COUNT = 383.6
        private const val BACK_ENCODER_COUNT = 753.2
    }
}


@Autonomous(name = "Blue")
class BlueAuto : ParkingAuto() {

    override fun onRun() {
        strafe(0.5, 1600L)
        moveForward(0.4, 1000L)
        strafe(0.3, 1800L)
    }
}

@Autonomous(name = "Red")
class RedAuto : ParkingAuto() {

    override fun onRun() {
        strafe(-0.5, 1600L)
        moveForward(0.4, 1000L)
        strafe(-0.3, 1800L)
    }
}
