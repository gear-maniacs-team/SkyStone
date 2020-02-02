package net.gearmaniacs.teamcode

import android.util.Log
import com.qualcomm.robotcore.hardware.HardwareMap
import net.gearmaniacs.teamcode.utils.IHardware
import net.gearmaniacs.teamcode.utils.IUpdatable
import net.gearmaniacs.teamcode.utils.getDevice
import org.openftc.revextensions2.ExpansionHubEx
import org.openftc.revextensions2.RevBulkData
import kotlin.concurrent.thread

class TeamRobot {

    private var hardwareInstances = emptyList<IHardware>()
    private var updatableInstances = emptyList<IUpdatable>()

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

    fun init(
        hardwareMap: HardwareMap,
        hardwareList: List<IHardware> = emptyList(),
        updatableList: List<IUpdatable> = emptyList()
    ) {
        isOpModeActive = true
        hardwareInstances = hardwareList
        updatableInstances = updatableList

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
            updatableInstances.forEach {
                it.update()
            }

            Thread.yield()
        }
    }

    fun stop() {
        isOpModeActive = false

        hardwareInstances.forEach {
            it.stop()
        }

        INSTANCE = null
    }

    fun updateExpansionHubs() {
        bulkInputData1 = expansionHub1.bulkInputData
        bulkInputData2 = expansionHub2.bulkInputData
    }

    companion object {
        private const val EXPANSION_HUB_1_NAME = "Expansion Hub 1"
        private const val EXPANSION_HUB_2_NAME = "Expansion Hub 2"

        @Volatile
        private var INSTANCE: TeamRobot? = null

        /*
         * This function should only be called after TeamRobot::init has been called
         * and before TeamRobot::stop was called
         *
         * If it is called at any other time, the function will throw an NPE
         */
        fun getRobot(): TeamRobot = INSTANCE!!

        fun getBulkData1() = getRobot().bulkInputData1

        fun getBulkData2() = getRobot().bulkInputData2

        init {
            Log.v("Gear Maniacs", "Loaded Library")
            System.loadLibrary("gear_maniacs")
        }
    }
}