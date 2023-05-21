package frc.robot.subsystems.elevator.command;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorConstants;

public class TriggerControl extends CommandBase {
    private final Elevator elevator = Elevator.getInstance();

    private final XboxController xbox;
    private final Trigger rightTrigger;
    private final Trigger leftTrigger;

    public TriggerControl(XboxController xbox) {
        this.xbox = xbox;
        this.rightTrigger = new Trigger(()-> xbox.getRightTriggerAxis()>0.2);
        this.leftTrigger = new Trigger(()-> xbox.getLeftTriggerAxis()>0.2);
        addRequirements(elevator);
    }

    @Override
    public void execute() {
        if (elevator.getPosition()>130845.0){
            elevator.setPower((-(xbox.getRightTriggerAxis()*ElevatorConstants.UP_POWER_MULTIPLIER)));
        }
        else {
            elevator.setPower((xbox.getLeftTriggerAxis() * ElevatorConstants.DOWN_POWER_MULTIPLIER) - (xbox.getRightTriggerAxis() * ElevatorConstants.UP_POWER_MULTIPLIER));
        }
    }

    @Override
    public void end(boolean interrupted) {
        elevator.setPower(0);
    }
}
