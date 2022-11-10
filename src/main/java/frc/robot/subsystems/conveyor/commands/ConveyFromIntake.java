package frc.robot.subsystems.conveyor.commands;

import frc.robot.subsystems.conveyor.Conveyor;

import java.util.function.DoubleSupplier;

public class ConveyFromIntake extends Convey {

    public ConveyFromIntake(DoubleSupplier power) {
        super(power, null, (output) -> Conveyor.getInstance().conveyFromIntake(output));
    }
}
