package org.firstinspires.ftc.teamcode.terminator

class PIDController(
    var kp: Double
)
{
    var setPoint = 0.0
    private var input = 0.0
    private var result = 0.0
    private var error = 0.0

    var minInput = 0.0
        private set
    var maxInput = 0.0
        private set
    var minOutput = 0.0
        private set
    var maxOutput = 0.0
        private set



    private fun calculate(){
        error = setPoint - input

        result = kp * error
    }

    fun performPID() : Double
    {
        calculate()
        return result
    }

    fun performPID(_input : Double) : Double
    {
        input = _input
        return performPID()
    }

    fun setInputRange(min: Double, max: Double) {
        require(min < max)

        minInput = min
        maxInput = max
    }

    fun setOutputRange(min: Double, max: Double) {
        require(min < max)

        minOutput = min
        maxOutput = max
    }
}