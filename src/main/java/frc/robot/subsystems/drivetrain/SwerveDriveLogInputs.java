package frc.robot.subsystems.drivetrain;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public class SwerveDriveLogInputs implements LoggableInputs {
    private static SwerveDriveLogInputs INSTANCE = null;
    public ChassisSpeeds speeds;
    public double[] pose;

    private SwerveDriveLogInputs() {
    }

    public static SwerveDriveLogInputs getInstance() {
        if (INSTANCE == null) {
            INSTANCE = new SwerveDriveLogInputs();
        }
        return INSTANCE;
    }

    @Override
    public void toLog(LogTable table) {
        table.put("velocityX", speeds.vxMetersPerSecond);
        table.put("velocityY", speeds.vyMetersPerSecond);
        table.put("velocityTheta", speeds.omegaRadiansPerSecond);
        table.put("pose", pose);
    }

    @Override
    public void fromLog(LogTable table) {
        speeds.vxMetersPerSecond = table.getDouble("velocityX", speeds.vxMetersPerSecond);
        speeds.vyMetersPerSecond = table.getDouble("velocityY", speeds.vyMetersPerSecond);
        speeds.omegaRadiansPerSecond = table.getDouble("velocityTheta", speeds.omegaRadiansPerSecond);
        pose = table.getDoubleArray("pose", pose);
    }
}
