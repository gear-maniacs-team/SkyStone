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
    @Volatile
    lateinit var bulkData1: RevBulkData
        private set
    @Volatile
    lateinit var bulkData2: RevBulkData
        private set

    var isOpModeActive = false
        private set
    var useBulkRead = true

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
        updateExpansionHubs()

        hardwareInstances.forEach {
            it.init(hardwareMap)
        }
    }

    fun start() {
        hardwareInstances.forEach {
            it.start()
        }

        if (updatableInstances.isNotEmpty() || useBulkRead)
            thread(block = ::updateAll)
    }

    private fun updateAll() {
        while (isOpModeActive) {
            updateExpansionHubs()

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

    private fun updateExpansionHubs() {
        if (!useBulkRead) return
        bulkData1 = expansionHub1.bulkInputData
        bulkData2 = expansionHub2.bulkInputData
    }

    companion object {
        private const val EXPANSION_HUB_1_NAME = "Expansion Hub 1"
        private const val EXPANSION_HUB_2_NAME = "Expansion Hub 2"

        @Volatile
        private var INSTANCE: TeamRobot? = null

        /**
         * This function will return null before TeamRobot::init has been called
         * and after TeamRobot::stop was called
         */
        fun getRobot(): TeamRobot? = INSTANCE

        fun getBulkData1() = getRobot()?.bulkData1

        fun getBulkData2() = getRobot()?.bulkData2

        fun isOpModeActive() = getRobot()?.isOpModeActive ?: false

        fun useBulkRead() = getRobot()?.useBulkRead ?: false

        init {
            Log.v("Gear Maniacs", "Loaded Library")
            System.loadLibrary("gear_maniacs")
        }
    }
}