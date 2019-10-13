package org.firstinspires.ftc.teamcode

import com.qualcomm.robotcore.hardware.HardwareMap
import kotlin.properties.Delegates

class TeamRobot {

    var x = 0
    var y = 0
    var isOpModeActive = false
        private set
    var wheels: WheelMotors by Delegates.notNull()
        private set
    val vuforia = VuforiaManager()

    fun init(hardwareMap: HardwareMap) {
        x = 0
        y = 0
        isOpModeActive = true
        wheels = WheelMotors(hardwareMap.dcMotor)

        INSTANCE = this
    }

    fun stop() {
        INSTANCE = null

        isOpModeActive = false
        vuforia.stopCamera()
    }

    companion object {
        private var INSTANCE: TeamRobot? = null

        /*
         * This function should only be called after TeamRobot::init has been called
         * and before TeamRobot::stop was called
         *
         * If it is called at any other time, the function will throw an NPE
         */
        fun getRobot(): TeamRobot = INSTANCE!!
    }
}