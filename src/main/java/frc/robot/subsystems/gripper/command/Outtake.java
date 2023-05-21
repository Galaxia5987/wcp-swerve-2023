package frc.robot.subsystems.gripper.command;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.gripper.Gripper;

public class Outtake extends CommandBase {
    private final Gripper gripper = Gripper.getInstance();
    private final double mainMotorPower;
    private final double spinPower;

    public Outtake(double mainMotorPower, double spinPower) {
        this.mainMotorPower = mainMotorPower;
        this.spinPower = spinPower;
        addRequirements(gripper);
    }

    @Override
    public void execute() {
        gripper.setSpinPower(spinPower);
        gripper.setMainMotorPower(mainMotorPower);
    }

    @Override
    public void end(boolean interrupted) {
        gripper.setMainMotorPower(0);
        gripper.setSpinPower(0);
    }
}
