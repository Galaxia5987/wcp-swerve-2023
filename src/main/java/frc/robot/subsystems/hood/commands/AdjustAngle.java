package frc.robot.subsystems.hood.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.hood.Hood;

import java.util.function.DoubleSupplier;

public class AdjustAngle extends CommandBase {
    private final Hood hood = Hood.getInstance();
    private final DoubleSupplier angle;

    public AdjustAngle(DoubleSupplier angle) {
        this.angle = angle;
    }

    @Override
    public void execute() {
        hood.setAngle(angle.getAsDouble());
    }

    @Override
    public void end(boolean interrupted) {
        hood.stop();
    }
}
