package net.gearmaniacs.teamcode.utils

/**
 * An boolean that delays the transition between values.
 */
class DelayedBoolean @JvmOverloads constructor(
    private val delay: Long,
    private var transitionTimestamp: Long = System.currentTimeMillis(),
    initialValue: Boolean = false
) {

    private var _value = initialValue

    val value: Boolean
        get() = _value

    fun invert() = updateAndGet(!value)

    fun update(timestamp: Long, newValue: Boolean) {
        val valueChanged = _value != newValue

        if (valueChanged && timestamp - transitionTimestamp > delay) {
            _value = newValue
            transitionTimestamp = timestamp
        }
    }

    fun update(newValue: Boolean) = update(System.currentTimeMillis(), newValue)

    fun updateAndGet(timestamp: Long, newValue: Boolean): Boolean {
        update(timestamp, newValue)
        return _value
    }

    fun updateAndGet(newValue: Boolean) = updateAndGet(System.currentTimeMillis(), newValue)
}