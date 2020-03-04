package net.gearmaniacs.teamcode.hardware.sensors.drivers.i2cEncoders

import com.qualcomm.robotcore.hardware.HardwareDevice
import com.qualcomm.robotcore.hardware.I2cAddr
import com.qualcomm.robotcore.hardware.I2cDeviceSynch
import com.qualcomm.robotcore.hardware.I2cDeviceSynch.ReadWindow
import com.qualcomm.robotcore.hardware.I2cDeviceSynchDevice
import com.qualcomm.robotcore.hardware.configuration.annotations.DeviceProperties
import com.qualcomm.robotcore.hardware.configuration.annotations.I2cDeviceType
import java.nio.ByteBuffer

@I2cDeviceType
@DeviceProperties(name = "I2C Encoder Sensor", description = "A loophole in the game manual being exploited", xmlTag = "EncoderSensor")
class I2CEncoderDriver(deviceClient: I2cDeviceSynch) : I2cDeviceSynchDevice<I2cDeviceSynch>(deviceClient, true), II2CInterface {

    private val encoder = I2CEncoderV2(this)

    init {
        super.registerArmingStateCallback(false)
        this.deviceClient.i2cAddress = I2cAddr.create7bit(0x3)
       // setOptimalReadWindow()
        super.registerArmingStateCallback(false)
        this.deviceClient.engage()

    }

    protected fun setOptimalReadWindow() { // Sensor registers are read repeatedly and stored in a register. This method specifies the
// registers and repeat read mode
        val readWindow = ReadWindow(
            I2CEncoderV2.EncoderRegistry.REG_GCONF.address,
            I2CEncoderV2.EncoderRegistry.LAST.address - I2CEncoderV2.EncoderRegistry.REG_GCONF.address + 1,
            I2cDeviceSynch.ReadMode.REPEAT
        )
        deviceClient.readWindow = readWindow
    }

    val currentPosition
        get() = encoder.readCounter()

    override fun doInitialize(): Boolean {
        return true
    }

    override fun getDeviceName(): String {
        return "I2C Encoder"
    }

    override fun getManufacturer(): HardwareDevice.Manufacturer {
        return HardwareDevice.Manufacturer.Unknown
    }

    override fun write8(address: Int, bValue: Int): Boolean {
        deviceClient.write8(address,bValue)
        return true
    }

    override fun read8(address: Int): Int {
        return deviceClient.read8(address).toInt()
    }

    override fun read32(address: Int): Int {
        return ByteBuffer.wrap(deviceClient.read(address,4)).int
    }

    override fun write32(address: Int, value: Int): Boolean {
        deviceClient.write(address,ByteBuffer.allocate(4).putInt(value).array())
        return true
    }
}