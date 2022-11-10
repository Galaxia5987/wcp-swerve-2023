package frc.robot.subsystems.drivetrain;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public class SwerveDriveLogInputs implements LoggableInputs {
    private static SwerveDriveLogInputs INSTANCE = null;
    public double velocityX;
    public double velocityY;
    public double velocityOmega;

    public double positionX;
    public double positionY;
    public double positionOmega;
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
        table.put("velocityX", velocityX);
        table.put("velocityY", velocityY);
        table.put("velocityOmega", velocityOmega);

        table.put("positionX", positionX);
        table.put("positionY", positionY);
        table.put("positionOmega", positionOmega);
        table.put("pose", pose);
    }

    @Override
    public void fromLog(LogTable table) {
        velocityX = table.getDouble("velocityX", velocityX);
        velocityY = table.getDouble("velocityY", velocityY);
        velocityOmega = table.getDouble("velocityOmega", velocityOmega);

        positionX = table.getDouble("positionX", positionX);
        positionY = table.getDouble("positionY", positionY);
        positionOmega = table.getDouble("positionOmega", positionOmega);
        pose = table.getDoubleArray("pose", pose);
    }
}
