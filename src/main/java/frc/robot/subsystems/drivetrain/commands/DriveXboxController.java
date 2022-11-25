package frc.robot.subsystems.drivetrain.commands;

import edu.wpi.first.wpilibj.XboxController;

import java.util.function.BooleanSupplier;

public class DriveXboxController extends HolonomicDrive {
    private final BooleanSupplier turnToTarget;

    public DriveXboxController(XboxController controller, BooleanSupplier turnToTarget) {
        super(controller::getLeftY, controller::getLeftX, controller::getRightX);
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
