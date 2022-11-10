package frc.robot.subsystems.conveyor;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Ports;
import frc.robot.subsystems.LoggedSubsystem;

public class Conveyor extends LoggedSubsystem {
    private static Conveyor INSTANCE = null;
    private final WPI_TalonFX motorFromIntake;
    private final WPI_TalonFX motorToShooter;
    private final DigitalInput preFlapBeamBreaker;
    private final ConveyorLogInputs inputs;

    private Conveyor() {
        super(ConveyorLogInputs.getInstance());
        inputs = ConveyorLogInputs.getInstance();

        motorFromIntake = new WPI_TalonFX(Ports.Conveyor.MOTOR_FROM_INTAKE);
        motorFromIntake.setInverted(TalonFXInvertType.Clockwise);
        motorFromIntake.setNeutralMode(NeutralMode.Brake);
        motorFromIntake.enableVoltageCompensation(true);
        motorFromIntake.configVoltageCompSaturation(10);

        motorToShooter = new WPI_TalonFX(Ports.Conveyor.MOTOR_TO_SHOOTER);
        motorToShooter.setInverted(TalonFXInvertType.CounterClockwise);
        motorToShooter.setNeutralMode(NeutralMode.Brake);
        motorToShooter.enableVoltageCompensation(true);
        motorToShooter.configVoltageCompSaturation(10);


        preFlapBeamBreaker = new DigitalInput(Ports.Conveyor.PRE_FLAP_BEAM);
    }

    public static Conveyor getInstance() {
        if (INSTANCE == null) {
            INSTANCE = new Conveyor();
        }
        return INSTANCE;
    }

    public boolean preFlapBeamSeesObject() {
        return inputs.preFBSensesObject;
    }

    public void conveyFromIntake(double power) {
        motorFromIntake.set(ControlMode.PercentOutput, power);
    }

    public void conveyToShooter(double power) {
        motorToShooter.set(ControlMode.PercentOutput, power);
    }

    public MotorsState getPower() {
        return new MotorsState(inputs.powerFromIntake, inputs.powerToShooter);
    }

    @Override
    public void updateInputs() {
        inputs.powerFromIntake = motorFromIntake.get();
        inputs.powerToShooter = motorToShooter.get();
        inputs.preFBSensedObject = inputs.preFBSensesObject;
        inputs.preFBSensesObject = !preFlapBeamBreaker.get();
    }

    @Override
    public String getSubsystemName() {
        return "Conveyor";
    }

    public static class MotorsState {
        public double motorFromIntakePower;
        public double motorToShooterPower;

        public MotorsState(double motorFromIntakePower, double motorToShooterPower) {
            this.motorFromIntakePower = motorFromIntakePower;
            this.motorToShooterPower = motorToShooterPower;
        }

        public double[] toArray() {
            return new double[]{motorFromIntakePower, motorToShooterPower};
        }
    }
}