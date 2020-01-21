package org.firstinspires.ftc.teamcode

import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.teamcode.detector.VuforiaManager
import org.firstinspires.ftc.teamcode.utils.IHardware
import org.firstinspires.ftc.teamcode.utils.IUpdatable
import org.firstinspires.ftc.teamcode.utils.PerformanceProfiler
import org.firstinspires.ftc.teamcode.utils.getDevice
import org.openftc.revextensions2.ExpansionHubEx
import org.openftc.revextensions2.RevBulkData
import java.lang.Exception
import java.util.concurrent.Executors
import java.util.concurrent.atomic.AtomicReference
import kotlin.concurrent.thread

class TeamRobot {

    private val performanceProfiler = PerformanceProfiler()
    private var hardwareInstances = emptyList<IHardware>()
    private var updatableInstances = emptyList<IUpdatable>()
    private var telemetry: Telemetry? = null

    // These fields are only initialized after init has been called
    lateinit var expansionHub1: ExpansionHubEx
        private set
    lateinit var expansionHub2: ExpansionHubEx
        private set
    lateinit var bulkInputData1: RevBulkData
        private set
    lateinit var bulkInputData2: RevBulkData
        private set

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

        expansionHub1 = hardwareMap.getDevice(EXPANSION_HUB_1_NAME)
        expansionHub2 = hardwareMap.getDevice(EXPANSION_HUB_2_NAME)

        hardwareInstances.forEach {
            it.init(hardwareMap)
        }
    }

    fun start() {
        hardwareInstances.forEach {
            it.start()
        }

        if (updatableInstances.isNotEmpty())
            thread(block = ::updateAll)
    }

    private fun updateAll() {
        while (isOpModeActive) {
            telemetry?.let {
                performanceProfiler.update(it, "Robot Thread Ms")
            }

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

    fun updateExpansionHubs() {
        bulkInputData1 = expansionHub1.bulkInputData
        bulkInputData2 = expansionHub2.bulkInputData
    }

    fun showPerformance(telemetry: Telemetry?) {
        this.telemetry = telemetry
    }

    companion object {
        private const val EXPANSION_HUB_1_NAME = "Expansion Hub 1"
        private const val EXPANSION_HUB_2_NAME = "Expansion Hub 2"

        private var INSTANCE: TeamRobot? = null

        /*
         * This function should only be called after TeamRobot::init has been called
         * and before TeamRobot::stop was called
         *
         * If it is called at any other time, the function will throw an NPE
         */
        private fun getRobot(): TeamRobot = INSTANCE!!

        fun getBulkData1() = getRobot().bulkInputData1

        fun getBulkData2() = getRobot().bulkInputData2
    }
}