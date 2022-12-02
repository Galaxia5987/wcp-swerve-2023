package frc.robot.subsystems.drivetrain.commands;

import edu.wpi.first.wpilibj.XboxController;

import java.util.function.BooleanSupplier;

public class DriveXboxController extends HolonomicDrive {
    private final BooleanSupplier turnToTarget;
    private final BooleanSupplier lock;
    private final BooleanSupplier robotOriented;

    public DriveXboxController(XboxController controller,
                               BooleanSupplier turnToTarget,
                               BooleanSupplier lock,
                               BooleanSupplier robotOriented) {
        super(controller::getLeftY, controller::getLeftX, controller::getRightX);
        this.turnToTarget = turnToTarget;
        this.lock = lock;
        this.robotOriented = robotOriented;
    }

    @Override
    public void execute() {
        swerveDrive.setFieldOriented(!robotOriented.getAsBoolean());
        if (turnToTarget.getAsBoolean()) {
            turnToTarget();
        } else if (lock.getAsBoolean()) {
            swerveDrive.lock();
        } else {
            super.execute();
        }
    }
}
