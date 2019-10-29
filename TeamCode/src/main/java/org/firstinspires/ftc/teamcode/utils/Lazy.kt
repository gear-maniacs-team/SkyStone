package org.firstinspires.ftc.teamcode.utils

private object UninitializedValue

private class FastLazyImpl<out T>(initializer: () -> T) : Lazy<T> {
    private var initializer: (() -> T)? = initializer
    private var _value: Any? = UninitializedValue

    override val value: T
        get() {
            if (_value === UninitializedValue) {
                _value = initializer!!()
                initializer = null
            }
            @Suppress("UNCHECKED_CAST")
            return _value as T
        }

    override fun isInitialized(): Boolean = _value !== UninitializedValue

    override fun toString(): String = if (isInitialized()) value.toString() else "Lazy value not initialized yet."
}

private class InitializedLazyImpl<out T>(override val value: T) : Lazy<T>  {

    override fun isInitialized(): Boolean = true

    override fun toString(): String = value.toString()
}

fun <T> fastLazy(initializer: () -> T): Lazy<T> = FastLazyImpl(initializer)