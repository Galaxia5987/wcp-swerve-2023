package frc.robot.subsystems.drivetrain;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public class SwerveModuleLogInputs implements LoggableInputs {
    private static final SwerveModuleLogInputs[] INSTANCES = new SwerveModuleLogInputs[]{null, null, null, null};
    public double aVelocity;
    public double aPosition;
    public double aAngle;
    public double aCurrent;

    public double dVelocity;
    public double dCurrent;

    private SwerveModuleLogInputs() {
    }

    public static SwerveModuleLogInputs getInstance(int wheel) {
        if (INSTANCES[wheel] == null) {
            INSTANCES[wheel] = new SwerveModuleLogInputs();
        }
        return INSTANCES[wheel];
    }

    @Override
    public void toLog(LogTable table) {
        table.put("aVelocity", aVelocity);
        table.put("aPosition", aPosition);
        table.put("aAngle", aAngle);
        table.put("aCurrent", aCurrent);

        table.put("dVelocity", dVelocity);
        table.put("dCurrent", dCurrent);
    }

    @Override
    public void fromLog(LogTable table) {
        aVelocity = table.getDouble("aVelocity", aVelocity);
        aPosition = table.getDouble("aPosition", aPosition);
        aAngle = table.getDouble("aAngle", aAngle);

        dVelocity = table.getDouble("dVelocity", dVelocity);
    }
}
