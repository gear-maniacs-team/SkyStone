package net.gearmaniacs.teamcode.autonomous

import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DcMotor
import net.gearmaniacs.teamcode.TeamRobot
import net.gearmaniacs.teamcode.hardware.motors.Wheels
import net.gearmaniacs.teamcode.hardware.sensors.Encoders

@TeleOp(name = "Ca'Ncer")
class ProMotionPlusMaxUltra2Elite : OpMode() {

    private val robot = TeamRobot()
    private val wheels = Wheels()
    private var previousVelocity = 0.0
    private val acceleration = 178 // cm / seconds^2
    private var startOfMotion = 0L

    override fun init() {
        robot.useBulkRead = false
        robot.init(
            hardwareMap,
            listOf(wheels)
        )

        wheels.setModeAll(DcMotor.RunMode.STOP_AND_RESET_ENCODER)
        wheels.setModeAll(DcMotor.RunMode.RUN_USING_ENCODER)
    }

    override fun start() {
        robot.start()
        startOfMotion = System.currentTimeMillis()
    }

    override fun loop() {
        val elapsedTime = (System.currentTimeMillis() - startOfMotion) / 1000.0
        val velocity = acceleration * elapsedTime
        wheels.setVelocityAll(cmToTicks(velocity))
        previousVelocity = velocity
    }

    override fun stop() {
        robot.stop()
    }

    companion object {
        const val TICKS = 537.6
        const val DIAMETER = 10
        const val MAX_RPM = 340
        const val MOTOR_VEL_F = 32767 / (MAX_RPM * TICKS / 60.0)

        fun cmToTicks(cm: Double) = cm * TICKS / (DIAMETER * Math.PI)
        fun ticksToCM(ticks: Double) = (DIAMETER * Math.PI * ticks) / TICKS
    }
}