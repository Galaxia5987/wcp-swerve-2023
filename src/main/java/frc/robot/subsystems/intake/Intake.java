package frc.robot.subsystems.intake;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import frc.robot.Ports;
import frc.robot.subsystems.LoggedSubsystem;

public class Intake extends LoggedSubsystem {
    private static Intake INSTANCE = null;

    private final WPI_TalonFX motor = new WPI_TalonFX(Ports.Intake.MOTOR);
    private final Solenoid solenoid = new Solenoid(PneumaticsModuleType.CTREPCM, Ports.Intake.SOLENOID);

    private final IntakeLogInputs inputs = IntakeLogInputs.getInstance();

    private double prevPower;

    private Intake() {
        super(IntakeLogInputs.getInstance());
        motor.configFactoryDefault();
        motor.setInverted(Ports.Intake.INVERSION);

        prevPower = 0;
    }

    public static Intake getInstance() {
        if (INSTANCE == null) {
            INSTANCE = new Intake();
        }
        return INSTANCE;
    }

    public void open() {
        solenoid.set(true);
    }

    public void close() {
        solenoid.set(false);
    }

    public double getPower() {
        return inputs.power;
    }

    public void setPower(double power) {
        motor.set(ControlMode.PercentOutput, power);
        prevPower = power;
    }

    public double getCurrent() {
        return inputs.current;
    }

    public boolean isOpen() {
        return inputs.isOpen;
    }

    public void stop() {
        motor.stopMotor();
    }

    @Override
    public void updateInputs() {
        inputs.isOpen = solenoid.get();
        inputs.current = motor.getSupplyCurrent();
        inputs.power = motor.get();
    }

    @Override
    public String getSubsystemName() {
        return "Intake";
    }
}
