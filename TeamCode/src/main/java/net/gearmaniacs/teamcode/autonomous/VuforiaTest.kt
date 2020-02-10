package net.gearmaniacs.teamcode.autonomous

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import net.gearmaniacs.teamcode.detector.VuforiaManager
import net.gearmaniacs.teamcode.utils.CpuUsage

@TeleOp(name = "CustomVuforiaTest")
class VuforiaTest : LinearOpMode() {

    override fun runOpMode() {
        val vuforia = VuforiaManager()
        vuforia.init(hardwareMap)
        vuforia.activateDetector()

        waitForStart()

        while (opModeIsActive()) {
            telemetry.addData("CPU Usage", CpuUsage.totalUsage)
            vuforia.recognitions.forEachIndexed { index, recognition ->
                telemetry.addData(index.toString(), "Top %f, Left %f", recognition.top, recognition.left)
            }
            telemetry.update()
            Thread.sleep(500)
        }

        vuforia.stop()
    }
}
