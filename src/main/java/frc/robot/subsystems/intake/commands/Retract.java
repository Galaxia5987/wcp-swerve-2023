package frc.robot.subsystems.intake.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.intake.IntakeConstants;
import frc.robot.subsystems.intake.IntakeIO;

public class Retract extends CommandBase {
    private final IntakeIO io;
    private final Mode mode;

    public Retract(IntakeIO io, Mode mode) {
        this.io = io;
        this.mode = mode;
    }

    @Override
    public void execute() {
        if (mode == Mode.UP) {
            io.setAngle(IntakeConstants.ANGLE_UP);
        } else {
            io.setAngle(IntakeConstants.ANGLE_DOWN);
        }
    }

    @Override
    public boolean isFinished() {
        return io.getAngleMotorCurrent() >= IntakeConstants.MAX_CURRENT;
    }

    @Override
    public void end(boolean interrupted) {
        if (mode == Mode.UP) {
            io.resetEncoder(0);
        } else if (mode == Mode.DOWN) {
            io.resetEncoder(-110.3); //TODO: check all values cuz they're probably in deg
        }
    }

    public enum Mode {
        UP,
        DOWN
    }
}
