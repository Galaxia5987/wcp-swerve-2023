package frc.robot.subsystems.gripper.command;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.gripper.Gripper;

public class Intake extends CommandBase {
    private final Gripper gripper = new Gripper();
    private final double mainMotorPower;
    private final double spinPower;
    private final boolean intake;

    public Intake(double mainMotorPower, double spinPower, boolean intake) {
        this.mainMotorPower = mainMotorPower;
        this.spinPower = spinPower;
        this.intake = intake;
        addRequirements(gripper);
    }

    @Override
    public void execute() {
        if (intake) {
            gripper.setMainMotorPower(mainMotorPower);
            gripper.setSpinPower(-spinPower);
        } else {
            gripper.setSpinPower(spinPower);
            gripper.setMainMotorPower(-mainMotorPower);
        }
    }

    @Override
    public void end(boolean interrupted) {
        gripper.setSpinPower(0);
        gripper.setMainMotorPower(0);
    }
}
