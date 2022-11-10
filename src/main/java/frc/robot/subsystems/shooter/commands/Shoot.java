package frc.robot.subsystems.shooter.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.shooter.Shooter;

import java.util.function.DoubleSupplier;

public class Shoot extends CommandBase {
    private final Shooter shooter = Shooter.getInstance();

    private final DoubleSupplier velocity;

    public Shoot(DoubleSupplier velocity) {
        this.velocity = velocity;
    }

    @Override
    public void execute() {
        shooter.setVelocity(velocity.getAsDouble());
    }

    @Override
    public void end(boolean interrupted) {
        shooter.stop();
    }
}
