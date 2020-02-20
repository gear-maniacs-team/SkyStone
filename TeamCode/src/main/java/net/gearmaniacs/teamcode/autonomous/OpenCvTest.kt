package net.gearmaniacs.teamcode.autonomous

import android.util.Log
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import net.gearmaniacs.teamcode.detector.OpenCvManager
import net.gearmaniacs.teamcode.detector.SkystoneDetector
import net.gearmaniacs.teamcode.utils.CpuUsage
import org.firstinspires.ftc.robotcore.internal.system.AppUtil
import org.openftc.opencvrepackaged.MD5
import java.io.File

@TeleOp(name = "OpenCvTest", group = "Vision")
class OpenCvTest : LinearOpMode() {

    override fun runOpMode() {
        val file = File(AppUtil.getInstance().rootActivity.filesDir, "extra/libOpenCvNative.so")
        file.parentFile?.listFiles()?.forEach { Log.v("FUCK", it.name) }
        file.delete()
        file.parentFile?.listFiles()?.forEach { Log.v("FUCK", it.name) }

        MD5.calculateMD5(file)

        return
        val pipeline = SkystoneDetector(telemetry)

        val manager = OpenCvManager(pipeline)
        manager.init(hardwareMap)
        manager.startDetector(720, 480)
        manager.showPreview(true)

        waitForStart()

        while (opModeIsActive()) {
            telemetry.addData("CPU Usage", CpuUsage.totalUsage)
            manager.printTelemetry(telemetry)
            telemetry.update()
            Thread.sleep(500)
        }

        manager.stopDetector()
    }
}