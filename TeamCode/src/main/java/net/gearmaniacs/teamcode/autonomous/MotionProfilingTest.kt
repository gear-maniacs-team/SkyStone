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
    private val controller = PIDFController(PIDCoefficients(0.0, 0.0, 0.0), 1 / MAX_VEL, 1 / MAX_ACC)
    private var startOfMotion = 0L

    override fun init() {
        robot.useBulkRead = false
        robot.init(
            hardwareMap,
            listOf(wheels, encoder),
            listOf(encoder)
        )

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
        robot.start()
        RobotPos.resetAll()
        startOfMotion = System.currentTimeMillis()
        controller.setOutputBounds(-1.0, 1.0)
    }

    override fun loop() {
        val elapsedTime = System.currentTimeMillis() - startOfMotion
        val temp = elapsedTime.toDouble() / 1000
        val state = motionProfile[temp]

        telemetry.addData("leftFront", wheels.leftFront.currentPosition)
        telemetry.addData("leftBack", wheels.leftBack.currentPosition)
        telemetry.addData("rightFront", wheels.rightFront.currentPosition)
        telemetry.addData("rightBack", wheels.rightBack.currentPosition)

        telemetry.addData("Elapsed Time", temp)
        telemetry.addData("State", state.toString())
        val dist =
            RobotPos.currentY  //ProMotionPlusMaxUltra2Elite.ticksToCM(wheels.rightFront.currentPosition.toDouble())
        telemetry.addData("Position", dist)

        val correctionInfo = controller.update(dist, state.v, state.a)
        controller.targetPosition = state.x
        telemetry.addData("Correction", correctionInfo)
        wheels.setPowerAll(correctionInfo)
        telemetry.update()
    }

    companion object {
        const val MAX_ACC = 118.0
        const val MAX_VEL = 178.0
    }
}
