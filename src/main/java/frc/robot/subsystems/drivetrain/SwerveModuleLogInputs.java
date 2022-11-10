package frc.robot.subsystems.drivetrain;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public class SwerveModuleLogInputs implements LoggableInputs {
    private static final SwerveModuleLogInputs[] INSTANCES = new SwerveModuleLogInputs[]{null, null, null, null};
    public double aVelocity;
    public Rotation2d aAngle;
    public double aPosition;
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
        table.put("aAngle", aAngle.getDegrees());
        table.put("aPosition", aPosition);
        table.put("aCurrent", aCurrent);

        table.put("dVelocity", dVelocity);
        table.put("dCurrent", dCurrent);
    }

    @Override
    public void fromLog(LogTable table) {
        aVelocity = table.getDouble("aVelocity", aVelocity);
        aAngle = Rotation2d.fromDegrees(table.getDouble("aAngle", aAngle.getDegrees()));
        aPosition = table.getDouble("aPosition", aPosition);
        aCurrent = table.getDouble("aCurrent", aCurrent);

        dVelocity = table.getDouble("dVelocity", dVelocity);
        dCurrent = table.getDouble("dCurrent", dCurrent);
    }
}
