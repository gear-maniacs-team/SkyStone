package org.firstinspires.ftc.teamcode

import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.teamcode.detector.VuforiaManager
import org.firstinspires.ftc.teamcode.utils.IHardware
import org.firstinspires.ftc.teamcode.utils.IUpdatable
import java.util.concurrent.Executors

class TeamRobot {

    private val updaterExecutor = Executors.newSingleThreadExecutor()
    private var hardwareInstances = emptyList<IHardware>()
    private var updatableInstances = emptyList<IUpdatable>()

    var isOpModeActive = false
        private set
    val vuforia = VuforiaManager()

    fun init(
        hardwareMap: HardwareMap,
        hardwareList: List<IHardware> = emptyList(),
        updatableList: List<IUpdatable> = emptyList()
    ) {
        isOpModeActive = true
        hardwareInstances = hardwareList
        updatableInstances = updatableList

        INSTANCE = this

        hardwareInstances.forEach {
            it.init(hardwareMap)
        }
    }

    fun start() {
        hardwareInstances.forEach {
            it.start()
        }

        if (updatableInstances.isNotEmpty())
            updaterExecutor.submit(::updateAll)
    }

    private fun updateAll() {
        while (isOpModeActive) {
            updatableInstances.forEach {
                it.update()
            }
            Thread.yield()
        }
    }

    fun stop() {
        INSTANCE = null

        hardwareInstances.forEach {
            it.stop()
        }

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