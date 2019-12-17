package org.firstinspires.ftc.teamcode.motors;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.utils.IHardware;
import org.jetbrains.annotations.NotNull;

public final class Wheels implements IHardware {

    public static double rpmToTps(double rpm, double encoder) {
        return rpm * (encoder / 60.0);
    }

    private DcMotorEx leftFront;
    private DcMotorEx leftBack;
    private DcMotorEx rightFront;
    private DcMotorEx rightBack;

    @Override
    public void init(@NotNull HardwareMap hardwareMap) {
        HardwareMap.DeviceMapping<DcMotor> dcMotors = hardwareMap.dcMotor;
        leftFront = (DcMotorEx) dcMotors.get("TL");
        leftBack = (DcMotorEx) dcMotors.get("BL");
        rightFront = (DcMotorEx) dcMotors.get("TR");
        rightBack = (DcMotorEx) dcMotors.get("BR");
    }

    @Override
    public void start() {
    }

    @Override
    public void stop() {
    }

    @NotNull
    public final DcMotorEx getLeftFront() {
        return leftFront;
    }

    @NotNull
    public final DcMotorEx getLeftBack() {
        return leftBack;
    }

    @NotNull
    public final DcMotorEx getRightFront() {
        return rightFront;
    }

    @NotNull
    public final DcMotorEx getRightBack() {
        return rightBack;
    }

    public final void setModeAll(@NotNull DcMotor.RunMode mode) {
        leftFront.setMode(mode);
        leftBack.setMode(mode);
        rightFront.setMode(mode);
        rightBack.setMode(mode);
    }

    public final void setPowerAll(double power) {
        leftFront.setPower(power);
        leftBack.setPower(power);
        rightFront.setPower(power);
        rightBack.setPower(power);
    }
}
