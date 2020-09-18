package net.gearmaniacs.teamcode.teleop

import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import net.gearmaniacs.teamcode.utils.RobotClock
import net.gearmaniacs.teamcode.utils.extensions.getDevice


@TeleOp(name = "MotorTester")
class MotorTester: OpMode() {

    private lateinit var motor: DcMotorEx

    private var lastTime = 0L
    private var lastPosition = 0

    private val magic = 383.6

    override fun init() {
        motor = hardwareMap.getDevice("right_rear")
        motor.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        motor.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
        lastTime = RobotClock.millis()
    }
    override fun loop() {
        Thread.sleep(50)
        motor.power = 1.0
        val deltaTime = (lastTime - RobotClock.millis()) / (1000.0 * 60.0)

        val currentPos = motor.currentPosition
        val speed = (currentPos - lastPosition) / magic / deltaTime

        telemetry.addData("Speed", speed)
        lastTime = RobotClock.millis()
        lastPosition = currentPos
    }
}