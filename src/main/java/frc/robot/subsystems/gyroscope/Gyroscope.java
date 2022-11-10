package frc.robot.subsystems.gyroscope;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.SPI;
import frc.robot.subsystems.LoggedSubsystem;

public class Gyroscope extends LoggedSubsystem {
    private final AHRS navx;
    private final GyroscopeLogInputs inputs;

    public Gyroscope() {
        super(GyroscopeLogInputs.getInstance());
        navx = new AHRS(SPI.Port.kMXP);
        inputs = GyroscopeLogInputs.getInstance();
        inputs.zeroAngle = new Rotation2d();
    }

    @Override
    public void updateInputs() {
        inputs.rawAngle = navx.getRotation2d();
        inputs.angle = getAngle();
    }

    @Override
    public String getSubsystemName() {
        return "Gyroscope";
    }

    /**
     * Resets the angle of the navx to the current angle.
     */
    public void resetAngle() {
        resetAngle(new Rotation2d());
    }

    /**
     * Resets the angle of the navx to the current angle.
     *
     * @param angle the angle in -180 to 180 degrees coordinate system.
     */
    public void resetAngle(Rotation2d angle) {
        inputs.zeroAngle = getRawAngle().minus(angle);
    }

    /**
     * Gets the current angle of the robot in respect to the start angle.
     *
     * @return the current angle of the robot in respect to the start angle.
     */
    public Rotation2d getAngle() {
        return getRawAngle().minus(inputs.zeroAngle);
    }

    /**
     * Gets the raw angle from the navx.
     *
     * @return the angle of the robot in respect to the angle of the robot initiation time.
     */
    public Rotation2d getRawAngle() {
        return inputs.rawAngle;
    }
}
