package net.gearmaniacs.teamcode.hardware.motors.type

import com.qualcomm.robotcore.hardware.configuration.DistributorInfo
import com.qualcomm.robotcore.hardware.configuration.ModernRoboticsMotorControllerParams
import com.qualcomm.robotcore.hardware.configuration.annotations.DeviceProperties
import com.qualcomm.robotcore.hardware.configuration.annotations.ExpansionHubPIDFPositionParams
import com.qualcomm.robotcore.hardware.configuration.annotations.ExpansionHubPIDFVelocityParams
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
/**
 * Please note that these Velocity PIDF Coefficients are calculated from our robot's
 * specific max velocity and may not work as well for you
 *
 * How these values were calculated:
 * * Find the max motor velocity (maxV) in ticks per second
 * * Calculate F: F = 32767 / maxV
 * * Calculate P: P = F / 10
 * * Calculate I: I = P / 10
 * * D remains 0
 */
@ExpansionHubPIDFVelocityParams(P = 1.293, I = 0.1293, D = 0.0, F = 12.93)
@ExpansionHubPIDFPositionParams(P = 5.0)
//@ModernRoboticsMotorControllerParams(P = 160, I = 32, D = 112, ratio = 25)
interface AndyMark3637
