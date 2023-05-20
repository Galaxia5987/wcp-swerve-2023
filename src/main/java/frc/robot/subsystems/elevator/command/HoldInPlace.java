package frc.robot.subsystems.elevator.command;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.elevator.Elevator;

public class HoldInPlace extends CommandBase {
    private final Elevator elevator = Elevator.getInstance();
    private final double position;

    public HoldInPlace() {
        this.position = elevator.getPosition();
        addRequirements(elevator);
    }

    @Override
    public void execute() {
        elevator.setPosition(position);
    }
}
