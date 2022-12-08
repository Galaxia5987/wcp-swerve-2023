package frc.robot.utils.math;

import edu.wpi.first.math.kinematics.ChassisSpeeds;

public class SwerveSpeedFilter {
    private double modelTolerance;
    private double sensorTolerance;

    public SwerveSpeedFilter(double modelTolerance, double sensorTolerance) {
        this.modelTolerance = modelTolerance;
        this.sensorTolerance = sensorTolerance;
    }

    public ChassisSpeeds calculate(ChassisSpeeds plant, ChassisSpeeds reference) {
        double vx = modelTolerance * plant.vxMetersPerSecond + sensorTolerance * reference.vxMetersPerSecond;
        double vy = modelTolerance * plant.vyMetersPerSecond + sensorTolerance * reference.vyMetersPerSecond;
        double omega = modelTolerance * plant.omegaRadiansPerSecond + sensorTolerance * reference.omegaRadiansPerSecond;

        return new ChassisSpeeds(
                vx / (modelTolerance + sensorTolerance),
                vy / (modelTolerance + sensorTolerance),
                omega / (modelTolerance + sensorTolerance));
    }

    public void setModelTolerance(double modelTolerance) {
        this.modelTolerance = modelTolerance;
    }

    public void setSensorTolerance(double sensorTolerance) {
        this.sensorTolerance = sensorTolerance;
    }
}
