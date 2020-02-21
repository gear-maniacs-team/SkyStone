package net.gearmaniacs.teamcode.autonomous

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import net.gearmaniacs.teamcode.detector.OpenCvManager
import net.gearmaniacs.teamcode.detector.SkystoneDetector
import net.gearmaniacs.teamcode.utils.CpuUsage
import java.lang.Exception

@TeleOp(name = "OpenCvTest", group = "Vision")
class OpenCvTest : LinearOpMode() {

    override fun runOpMode() {
        val pipeline = SkystoneDetector(telemetry)

        val manager = OpenCvManager(pipeline)
        manager.tryInitAndStart(hardwareMap)
        try {
            manager.init(hardwareMap)
            sleep(2000)
            manager.start()
        } catch (e: Exception) {
            e.printStackTrace()
            return
        }
        manager.showPreview(true)

        waitForStart()

        while (opModeIsActive()) {
            telemetry.addData("CPU Usage", CpuUsage.totalUsage)
            manager.printTelemetry(telemetry)
            telemetry.update()
            Thread.sleep(500)
        }

        manager.stop()
    }
}