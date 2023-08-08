package frc.robot.subsystems.intake;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import frc.robot.Constants;
import frc.robot.utils.units.UnitModel;

public class IntakeIOReal implements IntakeIO{

    private final TalonFX angleMotor;
    private final CANSparkMax spinMotor;

    private double angleSetpoint;

    private final UnitModel ticksPerRad = new UnitModel(IntakeConstants.TICKS_PER_RADIAN);

    public IntakeIOReal(int angleMotorID, int spinMotorID){
        this.angleMotor = new TalonFX(angleMotorID);
        this.spinMotor = new CANSparkMax(spinMotorID, CANSparkMaxLowLevel.MotorType.kBrushless);

        angleMotor.configFactoryDefault();
        spinMotor.restoreFactoryDefaults();

        angleMotor.configVoltageCompSaturation(IntakeConstants.VOLT_COMPENSATION_SATURATION);
        angleMotor.enableVoltageCompensation(true);
        angleMotor.configSupplyCurrentLimit(IntakeConstants.ANGLE_MOTOR_CURRENT_LIMIT);
        angleMotor.setInverted(Constants.CLOCKWISE);
        angleMotor.setNeutralMode(NeutralMode.Brake);
        angleMotor.config_kP(0, IntakeConstants.ANGLE_MOTOR_kP, Constants.TALON_TIMEOUT);
        angleMotor.config_kI(0, IntakeConstants.ANGLE_MOTOR_kI, Constants.TALON_TIMEOUT);
        angleMotor.config_kD(0, IntakeConstants.ANGLE_MOTOR_kD, Constants.TALON_TIMEOUT);
        angleMotor.config_kF(0, IntakeConstants.ANGLE_MOTOR_kF, Constants.TALON_TIMEOUT);

        spinMotor.enableVoltageCompensation(IntakeConstants.VOLT_COMPENSATION_SATURATION);
        spinMotor.setSmartCurrentLimit(IntakeConstants.SPIN_MOTOR_CURRENT_LIMIT);
        spinMotor.setInverted(true);
        spinMotor.setIdleMode(CANSparkMax.IdleMode.kCoast);
        spinMotor.burnFlash();
    }

    @Override
    public double getAngle() {
        return ticksPerRad.toUnits(angleMotor.getSelectedSensorPosition());
    }

    @Override
    public void setAngle(double angle) {
        angleSetpoint = angle;
        angleMotor.set(TalonFXControlMode.Position, angle);
    }

    @Override
    public double getSpinMotorPower() {
        return spinMotor.getEncoder().getVelocity();
    }

    @Override
    public void setSpinMotorPower(double power) {
        spinMotor.set(power);
    }

    @Override
    public void updateInputs(IntakeInputs inputs) {
        inputs.angle = getAngle();
        inputs.angleSetpoint = angleSetpoint;
        inputs.angleMotorVelocity = angleMotor.getSelectedSensorVelocity();
    }
}
