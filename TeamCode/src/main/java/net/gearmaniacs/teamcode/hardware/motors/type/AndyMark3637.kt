package net.gearmaniacs.teamcode.hardware.motors.type

import com.qualcomm.robotcore.hardware.configuration.DistributorInfo
import com.qualcomm.robotcore.hardware.configuration.ModernRoboticsMotorControllerParams
import com.qualcomm.robotcore.hardware.configuration.annotations.DeviceProperties
import com.qualcomm.robotcore.hardware.configuration.annotations.MotorType
import org.firstinspires.ftc.robotcore.external.navigation.Rotation

@Suppress("unused")
@MotorType(ticksPerRev = 537.6, gearing = 19.2, maxRPM = 340.0, orientation = Rotation.CW)
@DeviceProperties(xmlTag = "Am3637", name = "AndyMark 3637")
@DistributorInfo(
    distributor = "AndyMark",
    model = "am-3637",
    url = "https://www.andymark.com/products/neverest-orbital-20-gearmotor"
)
@ModernRoboticsMotorControllerParams(P = 160, I = 32, D = 112, ratio = 25)
interface AndyMark3637
