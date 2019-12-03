package org.firstinspires.ftc.teamcode

import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.teamcode.detector.VuforiaManager
import org.firstinspires.ftc.teamcode.motors.WheelMotors
import kotlin.properties.Delegates

class TeamRobot {

    // UPS Counter
    private var lastTime = 0L
    private var updatesCount = 0
    private var updateRate = 30f
    private var lastTelemetryItem: Telemetry.Item? = null

    var isOpModeActive = false
        private set
    var wheelsMotors by Delegates.notNull<WheelMotors>()
        private set
    val vuforia = VuforiaManager()

    fun init() {
        isOpModeActive = true
        lastTime = System.currentTimeMillis()

        INSTANCE = this
    }

    fun init(hardwareMap: HardwareMap) {
        isOpModeActive = true
        wheelsMotors = WheelMotors(hardwareMap.dcMotor)
        lastTime = System.currentTimeMillis()

        INSTANCE = this
    }

    fun update(telemetry: Telemetry) {
        val currentTime = System.currentTimeMillis()
        ++updatesCount

        val deltaTime = currentTime - lastTime
        if (deltaTime > 1000) { // if more than one second passed
            updateRate = (updatesCount * 0.5 + updateRate * 0.5).toFloat()
            updatesCount = 0
            lastTime = currentTime
        }

        lastTelemetryItem?.let {
            telemetry.removeItem(it)
        }
        lastTelemetryItem = telemetry.addData("UPS", updateRate)
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