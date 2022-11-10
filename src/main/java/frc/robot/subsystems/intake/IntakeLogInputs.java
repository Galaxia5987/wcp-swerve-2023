package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public class IntakeLogInputs implements LoggableInputs {
    private static IntakeLogInputs INSTACE = null;
    public boolean isOpen;
    public double current;
    public double power;

    private IntakeLogInputs() {
    }

    public static IntakeLogInputs getInstance() {
        if (INSTACE == null) {
            INSTACE = new IntakeLogInputs();
        }
        return INSTACE;
    }

    @Override
    public void toLog(LogTable table) {
        table.put("IsOpen", isOpen);
        table.put("Current", current);
        table.put("Power", power);
    }

    @Override
    public void fromLog(LogTable table) {
        isOpen = table.getBoolean("IsOpen", isOpen);
        current = table.getDouble("Current", current);
        power = table.getDouble("Power", power);
    }
}
