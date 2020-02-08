package net.gearmaniacs.teamcode.autonomous

import com.acmerobotics.roadrunner.control.PIDCoefficients
import com.acmerobotics.roadrunner.control.PIDFController
import com.acmerobotics.roadrunner.profile.MotionProfileGenerator
import com.acmerobotics.roadrunner.profile.MotionState
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.hardware.DcMotor
import net.gearmaniacs.teamcode.RobotPos
import net.gearmaniacs.teamcode.TeamOpMode
import net.gearmaniacs.teamcode.hardware.motors.Wheels
import net.gearmaniacs.teamcode.hardware.sensors.Encoders
import net.gearmaniacs.teamcode.utils.PerformanceProfiler
import net.gearmaniacs.teamcode.utils.getCurrentPosition

@Autonomous(name = "Motion Profiling")
class MotionProfilingTest : TeamOpMode() {

    private val performanceProfiler = PerformanceProfiler()
    private val encoder = Encoders()
    private val wheels = Wheels()
    private val controller = PIDFController(PIDCoefficients(0.0, 0.0, 0.0), 1 / MAX_VEL, 0.0)
    private var startOfMotion = 0L

    override fun init() {
        robot.init(
            hardwareMap,
            listOf(wheels, encoder),
            listOf(encoder)
        )
        RobotPos.resetAll()

        wheels.setModeAll(DcMotor.RunMode.STOP_AND_RESET_ENCODER)
        wheels.setModeAll(DcMotor.RunMode.RUN_USING_ENCODER)
    }

    private var setPoint = 200.0 // cm
    private val motionProfile = MotionProfileGenerator.generateSimpleMotionProfile(
        MotionState(0.0, 0.0, 0.0),
        MotionState(setPoint, 0.0, 0.0),
        MAX_VEL,
        MAX_ACC,
        100.0

    )

    override fun start() {
        super.start()
        startOfMotion = System.currentTimeMillis()
        controller.setOutputBounds(-1.0, 1.0)
    }

    override fun loop() {
        performanceProfiler.update(telemetry)
        val elapsedTime = System.currentTimeMillis() - startOfMotion
        val temp = elapsedTime.toDouble() / 1000
        val state = motionProfile[temp]

        telemetry.addData("leftFront", wheels.leftFront.getCurrentPosition(robot.bulkData2))
        telemetry.addData("leftBack", wheels.leftBack.getCurrentPosition(robot.bulkData2))
        telemetry.addData("rightFront", wheels.rightFront.getCurrentPosition(robot.bulkData1))
        telemetry.addData("rightBack", wheels.rightBack.getCurrentPosition(robot.bulkData1))
        telemetry.addLine("--")

        telemetry.addData("Elapsed Time", temp)
        telemetry.addData("State", state.toString())
        val dist = RobotPos.currentY
        telemetry.addData("Position", dist)

        controller.targetPosition = state.x
        val correctionInfo = controller.update(dist, state.v, state.a)
        telemetry.addData("Correction", correctionInfo)
        wheels.setPowerAll(correctionInfo)
    }

    companion object {
        const val MAX_ACC = 200.0
        const val MAX_VEL = 145.0 // 148 from tests, use 145 to be safe
    }
}
