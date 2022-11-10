package frc.robot.subsystems.gyroscope;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public class GyroscopeLogInputs implements LoggableInputs {
    private static GyroscopeLogInputs INSTANCE = null;

    public Rotation2d angle;
    public Rotation2d rawAngle;
    public Rotation2d zeroAngle;

    private GyroscopeLogInputs() {
    }

    public static GyroscopeLogInputs getInstance() {
        if (INSTANCE == null) {
            INSTANCE = new GyroscopeLogInputs();
        }
        return INSTANCE;
    }

    @Override
    public void toLog(LogTable table) {
        table.put("Angle", angle.getDegrees());
        table.put("RawAngle", rawAngle.getDegrees());
        table.put("ZeroAngle", zeroAngle.getDegrees());
    }

    @Override
    public void fromLog(LogTable table) {
        angle = Rotation2d.fromDegrees(table.getDouble("Angle", angle.getDegrees()));
        rawAngle = Rotation2d.fromDegrees(table.getDouble("RawAngle", rawAngle.getDegrees()));
        zeroAngle = Rotation2d.fromDegrees(table.getDouble("ZeroAngle", zeroAngle.getDegrees()));
    }
}
