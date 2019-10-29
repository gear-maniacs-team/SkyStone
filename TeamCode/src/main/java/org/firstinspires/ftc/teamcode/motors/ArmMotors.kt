package org.firstinspires.ftc.teamcode.motors

import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.HardwareMap

class ArmMotors(dcMotors: HardwareMap.DeviceMapping<DcMotor>) {

    val latchMotor: DcMotor = dcMotors["LatchMotor"]
    val armAngle: DcMotor = dcMotors["ArmAngle"]
    val armExtension: DcMotor = dcMotors["ArmExtension"]
    val collector: DcMotor = dcMotors["Collector"]
}
