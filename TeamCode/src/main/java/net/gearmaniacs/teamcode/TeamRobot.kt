package net.gearmaniacs.teamcode

import android.content.Context
import com.qualcomm.robotcore.eventloop.opmode.AnnotatedOpModeManager
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.eventloop.opmode.OpModeManagerNotifier.Notifications
import com.qualcomm.robotcore.eventloop.opmode.OpModeRegistrar
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.util.RobotLog
import net.gearmaniacs.teamcode.utils.IHardware
import net.gearmaniacs.teamcode.utils.IUpdatable
import net.gearmaniacs.teamcode.utils.extensions.getDevice
import net.gearmaniacs.teamcode.utils.extensions.justTry
import org.firstinspires.ftc.robotcore.internal.opmode.OpModeManagerImpl
import org.firstinspires.ftc.robotcore.internal.system.AppUtil
import org.firstinspires.ftc.robotcore.internal.ui.UILocation
import org.openftc.revextensions2.ExpansionHubEx
import org.openftc.revextensions2.RevBulkData
import kotlin.concurrent.thread

class TeamRobot(
    @Volatile
    var useBulkRead: Boolean = true
) {

    private var hardwareInstances = emptyList<IHardware>()
    private var updatableInstances = emptyList<IUpdatable>()

    // These fields are only initialized after init has been called
    lateinit var expansionHub1: ExpansionHubEx
        private set
    lateinit var expansionHub2: ExpansionHubEx
        private set
    @Volatile
    var bulkData1: RevBulkData? = null
        private set
    @Volatile
    var bulkData2: RevBulkData? = null
        private set

    var isOpModeActive = false
        private set

    fun init(
        hardwareMap: HardwareMap,
        hardwareList: List<IHardware> = emptyList(),
        updatableList: List<IUpdatable> = emptyList()
    ) {
        INSTANCE?.let {
            AppUtil.getInstance().showToast(UILocation.BOTH, "Another TeamRobot Instance already exists")

            // Try to stop the old TeamRobot Instance to avoid memory leaks
            justTry {
                it.stop()
            }

            INSTANCE = null
        }

        isOpModeActive = true
        hardwareInstances = hardwareList
        updatableInstances = updatableList

        INSTANCE = this

        expansionHub1 = hardwareMap.getDevice(EXPANSION_HUB_1_NAME)
        expansionHub2 = hardwareMap.getDevice(EXPANSION_HUB_2_NAME)
        if (useBulkRead)
            internalUpdateExpansionHubs()

        hardwareInstances.forEach {
            it.init(hardwareMap)
        }
    }

    fun start() {
        hardwareInstances.forEach {
            it.start()
        }

        if (updatableInstances.isNotEmpty() || useBulkRead)
            thread(block = ::updateAll, name = "Team Robot")
    }

    private fun updateAll() {
        while (isOpModeActive) {
            if (useBulkRead)
                internalUpdateExpansionHubs()

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

    private fun internalUpdateExpansionHubs() {
        val bulkInputData1: RevBulkData? = expansionHub1.bulkInputData
        val bulkInputData2: RevBulkData? = expansionHub2.bulkInputData

        if (bulkInputData1 == null) {
            RobotLog.ee(TAG, "Bulk Input Data 1 is null")
            return
        }

        if (bulkInputData2 == null) {
            RobotLog.ee(TAG, "Bulk Input Data 2 is null")
            return
        }

        bulkData1 = bulkInputData1
        bulkData2 = bulkInputData2
    }

    companion object {
        private const val TAG = "TeamRobot"

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

        /*
         * NOTE: We cannot simply pass `new OpModeNotifications()` inline to the call
         * to register the listener, because the SDK stores the list of listeners in
         * a WeakReference set. This causes the object to be garbage collected because
         * nothing else is holding a reference to it.
         */
        private val opModeNotifications = OpModeNotifications()

        @OpModeRegistrar
        @JvmStatic
        fun setupOpModeListenerOnStartRobot(
            context: Context?,
            manager: AnnotatedOpModeManager?
        ) {
            OpModeManagerImpl.getOpModeManagerOfActivity(AppUtil.getInstance().rootActivity).registerListener(
                opModeNotifications
            )
        }

        private class OpModeNotifications : Notifications {
            override fun onOpModePreInit(opMode: OpMode) = Unit

            override fun onOpModePreStart(opMode: OpMode) = Unit

            override fun onOpModePostStop(opMode: OpMode) {
                getRobot()?.let {
                    justTry { it.stop() }
                }
            }
        }
    }
}
