package org.firstinspires.ftc.teamcode.pid

import org.firstinspires.ftc.teamcode.RobotPos
import org.firstinspires.ftc.teamcode.utils.IUpdatable

class PidRotation : IUpdatable {

    val controller = PidController(153.6, 0.01, 0.01).apply {
        //setInputRange(-Math.PI, Math.PI)
        tolerance = 0.001
    }

    var debugEnabled: Boolean
        get() = controller.debugEnabled
        set(value) {
            controller.debugEnabled = value
        }

    @Volatile
    var output: Double = 0.0
        private set

    fun setOutputRange(min: Double, max: Double) {
        controller.setOutputRange(min, max)
    }

    override fun update() {
        val currentTarget = controller.setPoint
        val newAngle = RobotPos.targetAngle

        if (currentTarget != newAngle) {
            controller.reset()
            controller.setPoint = newAngle
        }

        output = controller.compute(RobotPos.currentAngle)
    }

    override fun toString(): String = controller.toString()
}
