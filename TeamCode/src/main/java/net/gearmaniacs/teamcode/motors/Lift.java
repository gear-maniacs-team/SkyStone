package net.gearmaniacs.teamcode.motors;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import net.gearmaniacs.teamcode.utils.IHardware;
import org.jetbrains.annotations.NotNull;

public final class Lift implements IHardware {

    private DcMotorEx left;
    private DcMotorEx right;

    @Override
    public void init(@NotNull HardwareMap hardwareMap) {
        HardwareMap.DeviceMapping<DcMotor> dcMotors = hardwareMap.dcMotor;
        left = (DcMotorEx) dcMotors.get("lift_left");
        right = (DcMotorEx) dcMotors.get("lift_right");

        left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        left.setTargetPosition(0);
        right.setTargetPosition(0);

        left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    @Override
    public final void start() {
    }

    @Override
    public final void stop() {
    }

    @NotNull
    public final DcMotorEx getLeft() {
        return left;
    }

    @NotNull
    public final DcMotorEx getRight() {
        return right;
    }
}
