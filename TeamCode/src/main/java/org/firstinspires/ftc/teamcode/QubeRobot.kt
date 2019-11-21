package org.firstinspires.ftc.teamcode

import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.teamcode.detector.VuforiaManager
import org.firstinspires.ftc.teamcode.motors.WheelMotors
import kotlin.properties.Delegates

class QubeRobot {

    var isOpModeActive = false
        private set
    var wheelsMotors by Delegates.notNull<WheelMotors>()
        private set
    val vuforia = VuforiaManager()

    fun init() {
        isOpModeActive = true

        INSTANCE = this
    }

    fun init(hardwareMap: HardwareMap) {
        isOpModeActive = true
        wheelsMotors = WheelMotors(hardwareMap.dcMotor)

        INSTANCE = this
    }

    fun stop() {
        INSTANCE = null

        isOpModeActive = false
        vuforia.stopCamera()
    }

    companion object {
        private var INSTANCE: QubeRobot? = null

        /*
         * This function should only be called after TeamRobot::init has been called
         * and before TeamRobot::stop was called
         *
         * If it is called at any other time, the function will throw an NPE
         */
        fun getRobot(): QubeRobot = INSTANCE!!
    }
}