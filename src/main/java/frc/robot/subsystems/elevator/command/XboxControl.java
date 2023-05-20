package frc.robot.subsystems.elevator.command;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.utils.Utils;

public class XboxControl extends CommandBase {
    private final Elevator elevator = Elevator.getInstance();
    private final XboxController xbox;

    public XboxControl(XboxController xbox) {
        this.xbox = xbox;
        addRequirements(elevator);
    }

    @Override
    public void execute() {
        elevator.setPower(-Utils.deadband(xbox.getLeftY(), 0.05));
    }

    @Override
    public void end(boolean interrupted) {
        elevator.setPower(0);
    }
}
