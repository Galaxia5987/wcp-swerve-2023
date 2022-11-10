package frc.robot.subsystems.conveyor.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.conveyor.Conveyor;

import java.util.function.DoubleConsumer;
import java.util.function.DoubleSupplier;

public class Convey extends CommandBase {
    private final DoubleSupplier power;
    private final DoubleConsumer conveyToShooter;
    private final DoubleConsumer conveyFromIntake;

    public Convey(DoubleSupplier power, DoubleConsumer conveyToShooter, DoubleConsumer conveyFromIntake) {
        this.power = power;
        this.conveyToShooter = conveyToShooter;
        this.conveyFromIntake = conveyFromIntake;
        addRequirements(Conveyor.getInstance());
    }

    @Override
    public void execute() {
        if (conveyToShooter != null) {
            conveyToShooter.accept(power.getAsDouble());
        }
        if (conveyFromIntake != null) {
            conveyFromIntake.accept(power.getAsDouble());
        }
    }

    @Override
    public void end(boolean interrupted) {
        if (conveyToShooter != null) {
            conveyToShooter.accept(0);
        }
        if (conveyFromIntake != null) {
            conveyFromIntake.accept(0);
        }
    }
}
