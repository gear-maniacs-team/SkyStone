package org.firstinspires.ftc.teamcode.autonomous

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.detector.VuforiaManager

@TeleOp(name = "CustomVuforiaTest")
class VuforiaTest : LinearOpMode() {

    override fun runOpMode() {
        val vuforia = VuforiaManager()
        vuforia.startDetectorAsync(hardwareMap)
        vuforia.waitForDetector()

        waitForStart()

        while (opModeIsActive()) {
            vuforia.recognitions.forEachIndexed { index, recognition ->
                telemetry.addData(index.toString(), "Top %f, Left %f", recognition.top, recognition.top)
            }
            telemetry.update()
        }
    }
}