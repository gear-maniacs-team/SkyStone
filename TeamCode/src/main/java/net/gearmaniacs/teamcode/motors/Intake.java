package net.gearmaniacs.teamcode.motors;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import net.gearmaniacs.teamcode.utils.IHardware;
import org.jetbrains.annotations.NotNull;

public final class Intake implements IHardware {

    private DcMotorEx left;
    private DcMotorEx right;

    @Override
    public void init(@NotNull HardwareMap hardwareMap) {
        HardwareMap.DeviceMapping<DcMotor> dcMotors = hardwareMap.dcMotor;
        left = (DcMotorEx) dcMotors.get("intake_left");
        right = (DcMotorEx) dcMotors.get("intake_right");
    }

    @Override
    public void start() {
    }

    @Override
    public void stop() {
    }

    @NotNull
    public final DcMotorEx getLeft() {
        return left;
    }

    @NotNull
    public final DcMotorEx getRight() {
        return right;
    }

    public final void setPowerAll(double power) {
        left.setPower(power);
        right.setPower(power);
    }
}
