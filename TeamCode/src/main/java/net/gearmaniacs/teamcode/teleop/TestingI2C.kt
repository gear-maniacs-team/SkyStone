package net.gearmaniacs.teamcode.teleop

import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DistanceSensor
import net.gearmaniacs.teamcode.TeamOpMode
import net.gearmaniacs.teamcode.hardware.sensors.drivers.i2cEncoders.I2CEncoderDriver
import net.gearmaniacs.teamcode.utils.PerformanceProfiler
import net.gearmaniacs.teamcode.utils.extensions.getDevice

@TeleOp(name = "I2C")
class TestingI2C : TeamOpMode() {

    private val performance = PerformanceProfiler()
    private lateinit var encoder: I2CEncoderDriver

    override fun init() {
        encoder = hardwareMap.getDevice("encoder")
        telemetry.msTransmissionInterval = 100
        encoder.reset()
    }

    override fun loop() {
        performance.update(telemetry)
        telemetry.addData("Black Magic", encoder.currentPosition)
    }
}
