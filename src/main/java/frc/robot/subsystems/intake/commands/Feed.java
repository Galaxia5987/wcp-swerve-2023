package frc.robot.subsystems.intake.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.intake.Intake;

import java.util.function.DoubleSupplier;

public class Feed extends CommandBase {
    private final Intake intake = Intake.getInstance();
    private final DoubleSupplier power;

    public Feed(DoubleSupplier power) {
        this.power = power;
    }

    @Override
    public void execute() {
        intake.setPower(power.getAsDouble());
    }

    @Override
    public void end(boolean interrupted) {
        intake.stop();
    }
}
