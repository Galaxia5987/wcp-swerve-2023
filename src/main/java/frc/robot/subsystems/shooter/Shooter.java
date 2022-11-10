package frc.robot.subsystems.shooter;

import com.ctre.phoenix.motorcontrol.ControlMode;
import frc.robot.Constants;
import frc.robot.Ports;
import frc.robot.subsystems.LoggedSubsystem;
import frc.robot.utils.motors.PIDTalon;
import frc.robot.utils.units.UnitModel;
import frc.robot.utils.units.Units;
import frc.robot.utils.valuetuner.WebConstant;

public class Shooter extends LoggedSubsystem {
    private static Shooter INSTANCE = null;

    private final PIDTalon motor;
    private final UnitModel unitModel = new UnitModel(Constants.Shooter.TICKS_PER_ROTATION);

    private final ShooterLogInputs inputs = ShooterLogInputs.getInstance();

    private final WebConstant webKp = WebConstant.of("Shooter", "kP", Constants.Shooter.Kp);
    private final WebConstant webKi = WebConstant.of("Shooter", "kI", Constants.Shooter.Ki);
    private final WebConstant webKd = WebConstant.of("Shooter", "kD", Constants.Shooter.Kd);
    private final WebConstant webKf = WebConstant.of("Shooter", "kF", Constants.Shooter.Kf);

    private Shooter() {
        super(ShooterLogInputs.getInstance());
        motor = new PIDTalon(Ports.Shooter.MOTOR);
        motor.configFactoryDefault();
        motor.setInverted(Ports.Shooter.INVERSION);

        motor.updatePID(0, webKp.get(), webKi.get(), webKd.get(), webKf.get());
    }

    public static Shooter getInstance() {
        if (INSTANCE == null) {
            INSTANCE = new Shooter();
        }
        return INSTANCE;
    }

    public void setPower(double power) {
        motor.set(power);
    }

    public double getVelocity() {
        return inputs.velocityRpm;
    }

    public void setVelocity(double velocity) {
        motor.set(ControlMode.Velocity, unitModel.toTicks100ms(Units.rpmToRps(velocity)));
        inputs.setpointRpm = velocity;
    }

    public double getCurrent() {
        return inputs.current;
    }

    public double getSetpoint() {
        return inputs.setpointRpm;
    }

    public void stop() {
        motor.stopMotor();
    }

    @Override
    public void updateInputs() {
        inputs.velocityRpm = Units.rpsToRpm(unitModel.toVelocity(motor.getSelectedSensorVelocity()));
        inputs.current = motor.getSupplyCurrent();
    }

    @Override
    public void periodic() {
        motor.updatePID(0, webKp.get(), webKi.get(), webKd.get(), webKf.get());
    }

    @Override
    public String getSubsystemName() {
        return "Shooter";
    }
}
