package frc.robot.subsystems.drivetrain.commands;

import edu.wpi.first.wpilibj.Joystick;

import java.util.function.BooleanSupplier;

public class DriveJoysticks extends HolonomicDrive {
    private final BooleanSupplier turnToTarget;

    public DriveJoysticks(Joystick leftJoystick, Joystick rightJoystick, BooleanSupplier turnToTarget) {
        super(() -> -leftJoystick.getY(), () -> -leftJoystick.getX(), rightJoystick::getX);
        this.turnToTarget = turnToTarget;
    }

    @Override
    public void execute() {
        if (turnToTarget.getAsBoolean()) {
            turnToTarget();
        } else {
            super.execute();
        }
    }
}
