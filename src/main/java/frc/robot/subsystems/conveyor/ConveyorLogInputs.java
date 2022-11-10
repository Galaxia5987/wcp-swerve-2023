package frc.robot.subsystems.conveyor;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public class ConveyorLogInputs implements LoggableInputs {
    private static ConveyorLogInputs INSTANCE = null;
    public double powerFromIntake;
    public double powerToShooter;
    public boolean preFBSensesObject;
    public boolean preFBSensedObject;

    private ConveyorLogInputs() {
    }

    public static ConveyorLogInputs getInstance() {
        if (INSTANCE == null) {
            INSTANCE = new ConveyorLogInputs();
        }
        return INSTANCE;
    }

    @Override
    public void toLog(LogTable table) {
        table.put("powerFromIntake", powerFromIntake);
        table.put("powerToShooter", powerToShooter);

        table.put("preFBSensesObject", preFBSensesObject);
        table.put("preFBSensedObject", preFBSensedObject);
    }

    @Override
    public void fromLog(LogTable table) {
        powerFromIntake = table.getDouble("powerFromIntake", powerFromIntake);
        powerToShooter = table.getDouble("powerToShooter", powerToShooter);
        preFBSensesObject = table.getBoolean("preFBSensesObject", preFBSensesObject);
        preFBSensedObject = table.getBoolean("preFBSensedObject", preFBSensedObject);
    }
}
