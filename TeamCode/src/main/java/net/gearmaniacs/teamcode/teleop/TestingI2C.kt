package net.gearmaniacs.teamcode.teleop

import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DistanceSensor
import net.gearmaniacs.teamcode.TeamOpMode
import net.gearmaniacs.teamcode.hardware.sensors.drivers.i2cEncoders.I2CEncoderDriver

@TeleOp(name = "I2C")
class TestingI2C: TeamOpMode() {

    lateinit var encoder: I2CEncoderDriver

    override fun init() {
        encoder = hardwareMap.get<I2CEncoderDriver>(I2CEncoderDriver::class.java, "encoder")
    }

    override fun loop() {
        telemetry.addData("Magic",encoder.currentPosition)
        telemetry.update()
    }
}