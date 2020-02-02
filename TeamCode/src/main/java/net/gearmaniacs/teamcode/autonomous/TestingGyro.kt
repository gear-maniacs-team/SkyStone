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
class TestingGyro : OpMode() {

    private val encoder = Encoders()
    private val robot = TeamRobot()
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

    lateinit var motionProfile: MotionProfile

    var previousTime = 0L
    var setPoint = Math.PI * 2

    override fun start() {
        robot.start()
        RobotPos.resetAll()
        motionProfile = MotionProfileGenerator.generateSimpleMotionProfile(
            MotionState(0.0, 0.0, 0.0),
            MotionState(setPoint, 0.0, 0.0),
            0.3,
            0.1,
            0.05
        )
        startOfMotion = System.currentTimeMillis()
    }

    override fun loop() {
        val elapsedTime = System.currentTimeMillis() - startOfMotion
        val temp = elapsedTime.toDouble() / 1000
        val state = motionProfile[temp]
        telemetry.addData("Elapsed Time", temp)
        telemetry.addData("state", "${state.x}, ${state.v},${state.a},${state.j}")
        telemetry.addData("Angle", RobotPos.currentAngle)
        val correctionInfo = controller.update(RobotPos.currentAngle, state.v, state.a)
        controller.targetPosition = state.x
        telemetry.addData("Correction", correctionInfo)
        wheels.setPowerAll(correctionInfo)
        telemetry.update()
    }
}
