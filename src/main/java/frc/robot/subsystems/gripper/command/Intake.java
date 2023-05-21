package frc.robot.subsystems.gripper.command;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.gripper.Gripper;

public class Intake extends CommandBase {
    private final Gripper gripper = new Gripper();
    private final double spinPower;
    private final double position;

    public Intake(double spinPower, double position) {
        this.spinPower = spinPower;
        this.position = position;
        addRequirements(gripper);
    }

    @Override
    public void execute() {
        gripper.setPosition(position);
        gripper.setSpinPower(-spinPower);
    }

    @Override
    public void end(boolean interrupted) {
        gripper.setSpinPower(0);
    }
}
