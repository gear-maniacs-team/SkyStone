package net.gearmaniacs.teamcode.autonomous

import com.acmerobotics.roadrunner.control.PIDCoefficients
import com.acmerobotics.roadrunner.control.PIDFController
import com.acmerobotics.roadrunner.profile.MotionProfile
import com.acmerobotics.roadrunner.profile.MotionProfileGenerator
import com.acmerobotics.roadrunner.profile.MotionState
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.hardware.DcMotor
import net.gearmaniacs.teamcode.RobotPos
import net.gearmaniacs.teamcode.TeamRobot
import net.gearmaniacs.teamcode.hardware.motors.Wheels
import net.gearmaniacs.teamcode.hardware.sensors.Encoders

@Autonomous(name = "Motion Profiling")
class MotionProfilingTest : OpMode() {

    private val robot = TeamRobot()
    private val encoder = Encoders()
    private val wheels = Wheels()
    private val controller = PIDFController(PIDCoefficients(0.0, 0.000, 0.0), 1.0, 1.0)
    private var startOfMotion = 0L

    override fun init() {
        robot.init(
            hardwareMap,
            listOf(wheels, encoder),
            listOf(encoder)
        )
        wheels.setModeAll(DcMotor.RunMode.RUN_WITHOUT_ENCODER)
    }

    private var setPoint = Math.PI * 2
    private val motionProfile = MotionProfileGenerator.generateSimpleMotionProfile(
        MotionState(0.0, 0.0, 0.0),
        MotionState(setPoint, 0.0, 0.0),
        0.3,
        0.1,
        0.05
    )

    override fun start() {
        robot.start()
        RobotPos.resetAll()
        startOfMotion = System.currentTimeMillis()
    }

    override fun loop() {
        val elapsedTime = System.currentTimeMillis() - startOfMotion
        val temp = elapsedTime.toDouble() / 1000
        val state = motionProfile[temp]

        telemetry.addData("Elapsed Time", temp)
        telemetry.addData("State", state.toString())
        telemetry.addData("Angle", RobotPos.currentAngle)

        val correctionInfo = controller.update(RobotPos.currentAngle, state.v, state.a)
        controller.targetPosition = state.x
        telemetry.addData("Correction", correctionInfo)
        wheels.setPowerAll(correctionInfo)
        telemetry.update()
    }
}
