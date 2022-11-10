package frc.robot.subsystems.conveyor.commands;

import frc.robot.subsystems.conveyor.Conveyor;

import java.util.function.DoubleSupplier;

public class ConveyAll extends Convey {

    public ConveyAll(DoubleSupplier power) {
        super(power, (output) -> Conveyor.getInstance().conveyToShooter(output),
                (output) -> Conveyor.getInstance().conveyFromIntake(output));
    }
}
