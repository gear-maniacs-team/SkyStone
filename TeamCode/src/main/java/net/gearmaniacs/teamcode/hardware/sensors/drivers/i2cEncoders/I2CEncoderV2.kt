package net.gearmaniacs.teamcode.hardware.sensors.drivers.i2cEncoders

import net.gearmaniacs.teamcode.hardware.sensors.drivers.i2cEncoders.I2CEncoderV2.ClockStrech.CLK_STRECH_DISABLE
import net.gearmaniacs.teamcode.hardware.sensors.drivers.i2cEncoders.I2CEncoderV2.Direction.RIGHT
import net.gearmaniacs.teamcode.hardware.sensors.drivers.i2cEncoders.I2CEncoderV2.EEPROM.BANK1
import net.gearmaniacs.teamcode.hardware.sensors.drivers.i2cEncoders.I2CEncoderV2.EncoderDataType.INT
import net.gearmaniacs.teamcode.hardware.sensors.drivers.i2cEncoders.I2CEncoderV2.EncoderType.STD
import net.gearmaniacs.teamcode.hardware.sensors.drivers.i2cEncoders.I2CEncoderV2.PullupConfig.ENABLE
import net.gearmaniacs.teamcode.hardware.sensors.drivers.i2cEncoders.I2CEncoderV2.RMODConfig.X1
import net.gearmaniacs.teamcode.hardware.sensors.drivers.i2cEncoders.I2CEncoderV2.WrapConfig.DISABLE

class I2CEncoderV2(val i2cInterface: II2CInterface) {
    private lateinit var dataType: EncoderDataType
    private var initializedGconf: Int = 0

    var onButtonRelease: (() -> Unit)? = null
    var onButtonPush: (() -> Unit)? = null
    var onButtonDoublePush: (() -> Unit)? = null
    var onIncrement: (() -> Unit)? = null
    var onDecrement: (() -> Unit)? = null
    var onChange: (() -> Unit)? = null
    var onMax: (() -> Unit)? = null
    var onMin: (() -> Unit)? = null
    var onMinMax: (() -> Unit)? = null
    var onGP1Rise: (() -> Unit)? = null
    var onGP1Fall: (() -> Unit)? = null
    var onGP2Rise: (() -> Unit)? = null
    var onGP2Fall: (() -> Unit)? = null
    var onGP3Rise: (() -> Unit)? = null
    var onGP3Fall: (() -> Unit)? = null
    var onFadeProcess: (() -> Unit)? = null

    companion object {
        private const val SLEEP_TIME_AFTER_RESET = 10L
    }

    fun begin(
        usedDataType: EncoderDataType = INT,
        wrapConfig: WrapConfig = DISABLE,
        direction: Direction = RIGHT,
        pullUpConfig: PullupConfig = ENABLE,
        rmodConfig: RMODConfig = X1,
        encoderType: EncoderType = STD,
        eepromBank: EEPROM = BANK1,
        clkStrech: ClockStrech = CLK_STRECH_DISABLE,
        relMode: RelMode = RelMode.REL_MODE_DISABLE
    ) {
        this.reset()
        Thread.sleep(SLEEP_TIME_AFTER_RESET)

        val gConfValue = usedDataType.bValue or wrapConfig.bValue or direction.bVal or pullUpConfig.bVal or
                rmodConfig.bVal or encoderType.bVal or eepromBank.bVal or clkStrech.bValue or relMode.bValue
        write8(EncoderRegistry.REG_GCONF, gConfValue and 0xFF)
        write8(EncoderRegistry.REG_GCONF2, (gConfValue shr 8) and 0xFF)
        this.initializedGconf = gConfValue
        this.dataType = usedDataType;
    }

    /**
     * Reset of the board. In this command there is 8ms delay in order to make the board correctly restart.
     */
    fun reset() {
        this.write8(EncoderRegistry.REG_GCONF, 0x80)
    }

    /**
     * Reads the encoder status and call the callbacks.
     * Returns true if the status of the encoder changed, otherwise return false
     */
    fun updateStatus(): Boolean {
        val status = this.readEncoderStatus();
        if (status == 0) {
            return false;
        }
        if (EncoderStatus.PUSHR.isSetIn(status)) {
            this.callEvent(this.onButtonRelease)
        }
        if (EncoderStatus.PUSHP.isSetIn(status)) {
            this.callEvent(this.onButtonPush)
        }
        if (EncoderStatus.PUSHD.isSetIn(status)) {
            this.callEvent(this.onButtonDoublePush)
        }
        if (EncoderStatus.RINC.isSetIn(status)) {
            this.callEvent(this.onIncrement)
            this.callEvent(this.onChange)
        }
        if (EncoderStatus.RDEC.isSetIn(status)) {
            this.callEvent(this.onDecrement)
            this.callEvent(this.onChange)
        }
        if (EncoderStatus.RMAX.isSetIn(status)) {
            this.callEvent(this.onMax)
            this.callEvent(this.onMinMax)
        }
        if (EncoderStatus.RMIN.isSetIn(status)) {
            this.callEvent(this.onMin)
            this.callEvent(this.onMinMax)
        }
        if (EncoderStatus.INT_2.isSetIn(status)) {
            val stat2 = readEncoderI2Status()
            if (stat2 == 0) {
                return true
            }

            if (EncoderI2Status.GP1_POS.isSetIn(stat2)) {
                this.callEvent(this.onGP1Rise)
            }
            if (EncoderI2Status.GP1_NEG.isSetIn(stat2)) {
                this.callEvent(this.onGP1Fall)
            }
            if (EncoderI2Status.GP2_POS.isSetIn(stat2)) {
                this.callEvent(this.onGP2Rise)
            }
            if (EncoderI2Status.GP2_NEG.isSetIn(stat2)) {
                this.callEvent(this.onGP2Fall)
            }
            if (EncoderI2Status.GP3_POS.isSetIn(stat2)) {
                this.callEvent(this.onGP3Rise)
            }
            if (EncoderI2Status.GP3_NEG.isSetIn(stat2)) {
                this.callEvent(this.onGP3Fall)
            }
            if (EncoderI2Status.FADE_INT.isSetIn(stat2)) {
                this.callEvent(this.onFadeProcess)
            }
        }
        return true
    }

    private fun callEvent(event: (() -> Unit)?) {
        event?.invoke()
    }

    fun autoconfigInterrupt() {
        var reg = 0
        if (this.onButtonRelease != null) {
            reg = reg or EncoderStatus.PUSHR.bValue
        }
        if (this.onButtonPush != null) {
            reg = reg or EncoderStatus.PUSHP.bValue
        }
        if (this.onButtonDoublePush != null) {
            reg = reg or EncoderStatus.PUSHD.bValue
        }
        if (this.onIncrement != null) {
            reg = reg or EncoderStatus.RINC.bValue
        }
        if (this.onDecrement != null) {
            reg = reg or EncoderStatus.RDEC.bValue
        }
        if (this.onChange != null) {
            reg = reg or EncoderStatus.RINC.bValue
            reg = reg or EncoderStatus.RDEC.bValue
        }
        if (this.onMax != null) {
            reg = reg or EncoderStatus.RMAX.bValue
        }
        if (this.onMin != null) {
            reg = reg or EncoderStatus.RMAX.bValue
        }
        if (this.onMinMax != null) {
            reg = reg or EncoderStatus.RMAX.bValue
            reg = reg or EncoderStatus.RMAX.bValue
        }
        if (this.onGP1Rise != null || this.onGP1Fall != null
            || this.onGP2Rise != null || this.onGP2Fall != null
            || this.onGP3Rise != null || this.onGP3Fall != null
            || this.onFadeProcess != null
        ) {
            reg = reg or EncoderStatus.INT_2.bValue
        }

        this.writeInterruptConfig(reg)
    }

    // ***************************
    // REGISTRY READ
    // ***************************
    fun readEncoderStatus(): Int {
        return this.read8(EncoderRegistry.REG_ESTATUS)
    }

    fun readEncoderI2Status(): Int {
        return this.read8(EncoderRegistry.REG_I2STATUS)
    }

    fun readGP1Conf(): Int {
        return this.read8(EncoderRegistry.REG_GP1CONF)
    }

    fun readGP2Conf(): Int {
        return this.read8(EncoderRegistry.REG_GP2CONF)
    }

    fun readGP3Conf(): Int {
        return this.read8(EncoderRegistry.REG_GP3CONF)
    }

    fun readInterruptConfig(): Int {
        return this.read8(EncoderRegistry.REG_INTCONF)
    }

    fun readLEDR(): Int {
        return this.read8(EncoderRegistry.REG_RLED)
    }

    fun readLEDG(): Int {
        return this.read8(EncoderRegistry.REG_GLED)
    }

    fun readLEDB(): Int {
        return this.read8(EncoderRegistry.REG_BLED)
    }

    fun readCounter(): Int {
        return this.read32(EncoderRegistry.REG_CVALB4)
    }

    fun readCounterFloat(): Float {
        return this.readFloat(EncoderRegistry.REG_CVALB4)
    }

    fun readMax(): Int {
        return this.read32(EncoderRegistry.REG_CMAXB4)
    }

    fun readMin(): Int {
        return this.read32(EncoderRegistry.REG_CMINB4)
    }

    fun readMaxFloat(): Float {
        return this.readFloat(EncoderRegistry.REG_CMAXB4)
    }

    fun readMinFloat(): Float {
        return this.readFloat(EncoderRegistry.REG_CMINB4)
    }

    fun readStep(): Int {
        return this.read32(EncoderRegistry.REG_ISTEPB4)
    }

    fun readStepFloat(): Float {
        return this.readFloat(EncoderRegistry.REG_ISTEPB4)
    }

    fun readGP1(): Int {
        return this.read8(EncoderRegistry.REG_GP1REG)
    }

    fun readGP2(): Int {
        return this.read8(EncoderRegistry.REG_GP2REG)
    }

    fun readGP3(): Int {
        return this.read8(EncoderRegistry.REG_GP3REG)
    }

    fun readAntiBouncingPeriod(): Int {
        return this.read8(EncoderRegistry.REG_ANTBOUNC)
    }

    fun readDoublePushPeriod(): Int {
        return this.read8(EncoderRegistry.REG_DPPERIOD)
    }

    fun readFadeRGB(): Int {
        return this.read8(EncoderRegistry.REG_FADERGB)
    }

    fun readFadeGP(): Int {
        return this.read8(EncoderRegistry.REG_FADEGP)
    }

    fun readFadeStatusRaw(): EncoderFadeStatus? {
        return EncoderFadeStatus.valueOf(this.read8(EncoderRegistry.REG_FSTATUS))
    }

    // ***************************
    // REGISTRY WRITE
    // ***************************

    fun writeGP1Conf(value: Int): Boolean {
        return this.write8(EncoderRegistry.REG_GP1CONF, value)
    }

    fun writeGP2Conf(value: Int): Boolean {
        return this.write8(EncoderRegistry.REG_GP2CONF, value)
    }

    fun writeGP3Conf(value: Int): Boolean {
        return this.write8(EncoderRegistry.REG_GP3CONF, value)
    }

    fun writeInterruptConfig(value: Int): Boolean {
        return this.write8(EncoderRegistry.REG_INTCONF, value)
    }

    fun writeLEDR(value: Int): Boolean {
        return this.write8(EncoderRegistry.REG_RLED, value)
    }

    fun writeLEDG(value: Int): Boolean {
        return this.write8(EncoderRegistry.REG_GLED, value)
    }

    fun writeLEDB(value: Int): Boolean {
        return this.write8(EncoderRegistry.REG_BLED, value)
    }

    fun writeCounter(value: Int): Boolean {
        return this.write32(EncoderRegistry.REG_CVALB4, value)
    }

    fun writeCounterFloat(value: Float): Boolean {
        return this.writeFloat(EncoderRegistry.REG_CVALB4, value)
    }

    fun writeMax(value: Int): Boolean {
        return this.write32(EncoderRegistry.REG_CMAXB4, value)
    }

    fun writeMin(value: Int): Boolean {
        return this.write32(EncoderRegistry.REG_CMINB4, value)
    }

    fun writeMaxFloat(value: Float): Boolean {
        return this.writeFloat(EncoderRegistry.REG_CMAXB4, value)
    }

    fun writeMinFloat(value: Float): Boolean {
        return this.writeFloat(EncoderRegistry.REG_CMINB4, value)
    }

    fun writeStep(value: Int): Boolean {
        return this.write32(EncoderRegistry.REG_ISTEPB4, value)
    }

    fun writeStepFloat(value: Float): Boolean {
        return this.writeFloat(EncoderRegistry.REG_ISTEPB4, value)
    }

    fun writeGP1(value: Int): Boolean {
        return this.write8(EncoderRegistry.REG_GP1REG, value)
    }

    fun writeGP2(value: Int): Boolean {
        return this.write8(EncoderRegistry.REG_GP2REG, value)
    }

    fun writeGP3(value: Int): Boolean {
        return this.write8(EncoderRegistry.REG_GP3REG, value)
    }

    /**
     * This method is used for writing the Anti-bouncing period ANTBOUNC.
     * This sets the period where an opposite rotation is ignored The I2C encoder V2 will
     * multiplies this value by 10 (value x10). Example: writeAntibouncingPeriod(20)  # Set an anti-bouncing of 200ms #
     */
    fun writeAntiBouncingPeriod(value: Int): Boolean {
        return this.write8(EncoderRegistry.REG_ANTBOUNC, value)
    }

    /**
     * This method is used for setting the window period DPPERIOD of the double push of the rotary encoder switch.
     * It the value is 0 the double push option is disabled. The I2C encoder V2 will multiply this value x10.
     * Example: writeDoublePushPeriod(50)  # Set a period for the double push of 500ms #
     */
    fun writeDoublePushPeriod(value: Int): Boolean {
        return this.write8(EncoderRegistry.REG_DPPERIOD, value)
    }

    fun writeFadeRGB(value: Int): Boolean {
        return this.write8(EncoderRegistry.REG_FADERGB, value)
    }

    fun writeFadeGP(value: Int): Boolean {
        return this.write8(EncoderRegistry.REG_FADEGP, value)
    }

    // ***************************
    // PRIVATE METHODS
    // ***************************

    private fun write8(register: EncoderRegistry, bValue: Int): Boolean {
        return this.i2cInterface.write8(register.address, bValue)
    }

    private fun write32(register: EncoderRegistry, value: Int): Boolean {
        return this.i2cInterface.write32(register.address, value)
    }

    private fun writeFloat(registry: EncoderRegistry, value: Float): Boolean {
        return this.i2cInterface.writeFloat(registry.address, value);
    }

    private fun read8(register: EncoderRegistry): Int {
        return this.i2cInterface.read8(register.address)
    }

    private fun read32(register: EncoderRegistry): Int {
        return this.i2cInterface.read32(register.address)
    }

    private fun readFloat(register: EncoderRegistry): Float {
        return this.i2cInterface.readFloat(register.address)
    }

    // ***************************
    // ENUMERATIONS
    // ***************************

    enum class EEPROM(val bVal: Int) {
        /** Select the first EEPROM bank */
        BANK1(0x40),
        /** Select the second EEPROM bank */
        BANK2(0x00)
    }

    enum class EncoderType(val bVal: Int) {
        /** Standard rotary encoder is soldered */
        STD(0x00),
        /** RGB illuminated encoder is soldered */
        RGB(0x20)
    }

    enum class RMODConfig(val bVal: Int) {
        /** Encoder in X1 mode */
        X1(0x00),
        /** Encoder in X2 mode */
        X2(0x10)
    }

    enum class PullupConfig(val bVal: Int) {
        /** Enable the internal pull-up on the INT pin */
        ENABLE(0x00),
        /** Disable the internal pull-up on the INT pin */
        DISABLE(0x08)
    }

    enum class Direction(val bVal: Int) {
        /** Rotate left side to increase the value counter */
        LEFT(0x04),
        /** Rotate right side to increase the value counter */
        RIGHT(0x00)
    }

    enum class WrapConfig(val bValue: Int) {
        /**
         * Wrap enable. When the counter value reaches the CMAX+1, restart to the CMIN and vice versa
         */
        ENABLE(0x02),
        /**
         * Wrap disable. When the counter value reaches the CMAX or CMIN, the counter stops to increasing or decreasing
         */
        DISABLE(0x00)
    }

    enum class ClockStrech(val bValue: Int) {
        CLK_STRECH_ENABLE(0x0100),
        CLK_STRECH_DISABLE(0x0000)
    }

    enum class RelMode(val bValue: Int) {
        REL_MODE_ENABLE(0x0200),
        REL_MODE_DISABLE(0x0000)
    }

    enum class EncoderDataType(val bValue: Int) {
        /** The Threshold, counter step and counter value are used with floating numbers */
        FLOAT(0x01),
        /** The Threshold, counter step and counter value are used with integer numbers */
        INT(0x00)
    }

    enum class EncoderRegistry(val address: Int) {
        REG_GCONF(0x00),
        REG_GP1CONF(0x01),
        REG_GP2CONF(0x02),
        REG_GP3CONF(0x03),
        REG_INTCONF(0x04),
        REG_ESTATUS(0x05),
        REG_I2STATUS(0x06),
        REG_FSTATUS(0x07),
        REG_CVALB4(0x08),
        REG_CVALB3(0x09),
        REG_CVALB2(0x0A),
        REG_CVALB1(0x0B),
        REG_CMAXB4(0x0C),
        REG_CMAXB3(0x0D),
        REG_CMAXB2(0x0E),
        REG_CMAXB1(0x0F),
        REG_CMINB4(0x10),
        REG_CMINB3(0x11),
        REG_CMINB2(0x12),
        REG_CMINB1(0x13),
        REG_ISTEPB4(0x14),
        REG_ISTEPB3(0x15),
        REG_ISTEPB2(0x16),
        REG_ISTEPB1(0x17),
        REG_RLED(0x18),
        REG_GLED(0x19),
        REG_BLED(0x1A),
        REG_GP1REG(0x1B),
        REG_GP2REG(0x1C),
        REG_GP3REG(0x1D),
        REG_ANTBOUNC(0x1E),
        REG_DPPERIOD(0x1F),
        REG_FADERGB(0x20),
        REG_FADEGP(0x21),
        REG_GAMRLED(0x27),
        REG_GAMGLED(0x28),
        REG_GAMBLED(0x29),
        REG_GAMMAGP1(0x2A),
        REG_GAMMAGP2(0x2B),
        REG_GAMMAGP3(0x2C),
        REG_GCONF2(0x30),
        REG_IDCODE(0x70),
        REG_VERSION(0x71),
        REG_EEPROMS(0x80)
    }

    enum class EncoderFadeStatus(val bValue: Int) {
        NULL(0x00),
        FADE_R(0x01),
        FADE_G(0x02),
        FADE_B(0x04),
        FADES_GP1(0x08),
        FADES_GP2(0x10),
        FADES_GP3(0x20);

        companion object {
            fun valueOf(value: Int): EncoderFadeStatus? = EncoderFadeStatus.values().find { it.bValue == value }
        }
    }

    enum class EncoderStatus(val bValue: Int) {
        PUSHR(0x01),
        PUSHP(0x02),
        PUSHD(0x04),
        RINC(0x08),
        RDEC(0x10),
        RMAX(0x20),
        RMIN(0x40),
        INT_2(0x80);

        fun isSetIn(status: Int): Boolean {
            return (status and bValue) != 0
        }
    }

    enum class EncoderI2Status(val bValue: Int) {
        GP1_POS(0x01),
        GP1_NEG(0x02),
        GP2_POS(0x04),
        GP2_NEG(0x08),
        GP3_POS(0x10),
        GP3_NEG(0x20),
        FADE_INT(0x40);

        fun isSetIn(status: Int): Boolean {
            return (status and bValue) != 0
        }
    }
}