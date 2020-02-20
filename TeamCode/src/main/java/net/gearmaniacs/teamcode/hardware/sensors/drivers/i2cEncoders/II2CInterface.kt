package net.gearmaniacs.teamcode.hardware.sensors.drivers.i2cEncoders

interface II2CInterface {
    fun write8(address: Int, bValue: Int): Boolean
    fun read8(address: Int): Int
    fun read32(address: Int): Int
    fun write32(address: Int, value: Int): Boolean
}
