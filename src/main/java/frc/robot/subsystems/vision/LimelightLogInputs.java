package frc.robot.subsystems.vision;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public class LimelightLogInputs implements LoggableInputs {
    public static LimelightLogInputs INSTANCE = null;
    public boolean hasTargets;
    public double estimatedYaw;
    public double estimatedDistance;

    private LimelightLogInputs() {
    }

    public static LimelightLogInputs getInstance() {
        if (INSTANCE == null) {
            INSTANCE = new LimelightLogInputs();
        }
        return INSTANCE;
    }

    @Override
    public void toLog(LogTable table) {
        table.put("hasTargets", hasTargets);
        table.put("estimatedYaw", estimatedYaw);
        table.put("estimatedDistance", estimatedDistance);
    }

    @Override
    public void fromLog(LogTable table) {
        hasTargets = table.getBoolean("hasTargets", hasTargets);
        estimatedYaw = table.getDouble("estimatedYaw", estimatedYaw);
        estimatedDistance = table.getDouble("estimatedDistance", estimatedDistance);
    }
}
