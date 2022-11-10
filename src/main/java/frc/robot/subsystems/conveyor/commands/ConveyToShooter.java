package frc.robot.subsystems.conveyor.commands;

import frc.robot.subsystems.conveyor.Conveyor;

import java.util.function.DoubleSupplier;

public class ConveyToShooter extends Convey {

    public ConveyToShooter(DoubleSupplier power) {
        super(power, (output) -> Conveyor.getInstance().conveyToShooter(output), null);
    }
}
