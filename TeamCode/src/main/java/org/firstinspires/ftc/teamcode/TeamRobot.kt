package org.firstinspires.ftc.teamcode

import com.qualcomm.robotcore.hardware.HardwareMap
import kotlin.properties.Delegates

class TeamRobot {

    var x = 0
    var y = 0
    var isOpModeActive = false
    var wheels: WheelMotors by Delegates.notNull()
        private set

    val vuforiaManager = VuforiaManager()

    fun onInit(hardwareMap: HardwareMap) {
        x = 0
        y = 0
        isOpModeActive = false
        wheels = WheelMotors(hardwareMap.dcMotor)
    }
}