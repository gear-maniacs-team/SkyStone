package org.firstinspires.ftc.teamcode.rover_ruckus

import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.HardwareMap

@Deprecated("Used in previous Season")
class ArmMotors(dcMotors: HardwareMap.DeviceMapping<DcMotor>) {

    val latchMotor: DcMotor = dcMotors["LatchMotor"]
    val armAngle: DcMotor = dcMotors["ArmAngle"]
    val armExtension: DcMotor = dcMotors["ArmExtension"]
    val collector: DcMotor = dcMotors["Collector"]
}
