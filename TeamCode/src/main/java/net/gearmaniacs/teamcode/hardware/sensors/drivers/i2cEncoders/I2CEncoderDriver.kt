package net.gearmaniacs.teamcode.hardware.sensors.drivers.i2cEncoders

import android.util.Log
import com.qualcomm.robotcore.hardware.HardwareDevice
import com.qualcomm.robotcore.hardware.I2cAddr
import com.qualcomm.robotcore.hardware.I2cDeviceSynch
import com.qualcomm.robotcore.hardware.I2cDeviceSynch.ReadWindow
import com.qualcomm.robotcore.hardware.I2cDeviceSynchDevice
import com.qualcomm.robotcore.hardware.configuration.annotations.DeviceProperties
import com.qualcomm.robotcore.hardware.configuration.annotations.I2cDeviceType
import java.nio.ByteBuffer

@I2cDeviceType
@DeviceProperties(
    name = "I2C Encoder Sensor",
    description = "A loophole in the game manual being exploited",
    xmlTag = "EncoderSensor"
)
class I2CEncoderDriver(deviceClient: I2cDeviceSynch) : I2cDeviceSynchDevice<I2cDeviceSynch>(deviceClient, true),
    II2CInterface {

    private val encoder = I2CEncoderV2(this)

    init {
        this.deviceClient.i2cAddress = I2cAddr.create7bit(0x3)
        super.registerArmingStateCallback(false)
        this.deviceClient.engage()

        encoder.begin()
        encoder.writeMin(Int.MIN_VALUE)
        encoder.writeMax(Int.MAX_VALUE)
    }

    @Synchronized
    override fun doInitialize(): Boolean = true

    override fun getDeviceName(): String = "I2C Encoder"

    override fun getManufacturer() = HardwareDevice.Manufacturer.Unknown

    override fun write8(address: Int, bValue: Int): Boolean {
        deviceClient.write8(address, bValue)
        return true
    }

    override fun read8(address: Int): Int {
        return deviceClient.read8(address).toInt()
    }

    override fun read32(address: Int): Int {
        return ByteBuffer.wrap(deviceClient.read(address, 4)).int
    }

    override fun write32(address: Int, value: Int): Boolean {
        deviceClient.write(address, ByteBuffer.allocate(4).putInt(value).array())
        return true
    }

    val currentPosition
        get() = encoder.readCounter()

    fun reset() {
        encoder.writeCounter(0)
        Thread.sleep(10)
    }
}
