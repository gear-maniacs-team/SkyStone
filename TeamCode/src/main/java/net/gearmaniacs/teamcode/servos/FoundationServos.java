package net.gearmaniacs.teamcode.servos;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import net.gearmaniacs.teamcode.utils.IHardware;
import org.jetbrains.annotations.NotNull;

public class FoundationServos implements IHardware {

    private Servo left;
    private Servo right;

    @Override
    public void init(@NotNull HardwareMap hardwareMap) {
        left = hardwareMap.get(Servo.class, "foundation_left");
        right = hardwareMap.get(Servo.class, "foundation_right");
    }

    @NotNull
    public final Servo getLeft() {
        return left;
    }

    @NotNull
    public final Servo getRight() {
        return right;
    }

    public final void attach() {
        left.setPosition(0.0);
        right.setPosition(1.0);
    }

    public final void detach() {
        left.setPosition(1.0);
        right.setPosition(0.0);
    }

    @Override
    public void start() {
    }

    @Override
    public void stop() {
    }
}
