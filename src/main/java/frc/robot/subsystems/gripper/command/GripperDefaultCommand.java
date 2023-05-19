package frc.robot.subsystems.gripper.command;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.gripper.Gripper;

public class GripperDefaultCommand extends CommandBase {
    private final Gripper gripper = new Gripper();
    private final double mainMotorPower;
    private final double spinPower;
    private final boolean intake;

    public GripperDefaultCommand(double mainMotorPower, double spinPower, boolean intake) {
        this.mainMotorPower = mainMotorPower;
        this.spinPower = spinPower;
        this.intake = intake;
        addRequirements(gripper);
    }

    @Override
    public void execute() {
        if (intake) {
            gripper.setMainMotorPower(mainMotorPower);
            gripper.setSpinPower(spinPower);
        }
        else {
            gripper.setSpinPower(-spinPower);
        }
    }

    @Override
    public void end(boolean interrupted) {
        gripper.setSpinPower(0);
        gripper.setMainMotorPower(0);
    }
}
