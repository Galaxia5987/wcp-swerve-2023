package frc.robot.subsystems.drivetrain;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import frc.robot.utils.math.AngleUtil;
import frc.robot.utils.units.UnitModel;

public class ModuleIOReal implements ModuleIO {

    private final CANSparkMax mainDriveMotor;
    private final CANSparkMax auxDriveMotor;
    private final CANSparkMax angleMotor;

    private final DutyCycleEncoder encoder;

    private final PIDController drivePID = new PIDController(SwerveConstants.MAIN_DRIVE_kP, SwerveConstants.MAIN_DRIVE_kI, SwerveConstants.MAIN_DRIVE_kD);
    private final PIDController anglePID = new PIDController(SwerveConstants.ANGLE_kP, SwerveConstants.ANGLE_kI, SwerveConstants.ANGLE_kD);

    private final UnitModel ticksPerRad = new UnitModel(SwerveConstants.TICKS_PER_RADIAN);
    private final UnitModel ticksPerMeter = new UnitModel(SwerveConstants.TICKS_PER_METER);
    private final int number;

    private double angleSetpoint;
    private double currentAngle;
    private double angleMotorPosition;
    private double driveMotorVelocitySetpoint;

    public ModuleIOReal(int mainDriveMotorID, int auxDriveMotorID, int angleMotorID,
                        int encoderID, boolean driveInvert, boolean angleInvert, int number) {

        this.mainDriveMotor = new CANSparkMax(mainDriveMotorID, CANSparkMaxLowLevel.MotorType.kBrushless);
        this.auxDriveMotor = new CANSparkMax(auxDriveMotorID, CANSparkMaxLowLevel.MotorType.kBrushless);
        this.angleMotor = new CANSparkMax(angleMotorID, CANSparkMaxLowLevel.MotorType.kBrushless);

        this.encoder = new DutyCycleEncoder(encoderID);

        this.number = number;

        mainDriveMotor.restoreFactoryDefaults();
        auxDriveMotor.restoreFactoryDefaults();
        angleMotor.restoreFactoryDefaults();

        mainDriveMotor.enableVoltageCompensation(SwerveConstants.VOLT_COMP_SATURATION);
        mainDriveMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
        mainDriveMotor.setSmartCurrentLimit(SwerveConstants.DRIVE_MOTOR_CURRENT_LIMIT);
        mainDriveMotor.setInverted(driveInvert);

        auxDriveMotor.follow(mainDriveMotor); //TODO: check if should oppose
        auxDriveMotor.enableVoltageCompensation(SwerveConstants.VOLT_COMP_SATURATION);
        auxDriveMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
        auxDriveMotor.setSmartCurrentLimit(SwerveConstants.DRIVE_MOTOR_CURRENT_LIMIT);

        angleMotor.enableVoltageCompensation(SwerveConstants.VOLT_COMP_SATURATION);
        angleMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
        angleMotor.setSmartCurrentLimit(SwerveConstants.ANGLE_MOTOR_CURRENT_LIMIT);
        angleMotor.setInverted(angleInvert);
    }

    @Override
    public void updateInputs(SwerveModuleInputs inputs) {
        inputs.absolutePosition = encoder.getAbsolutePosition();

        inputs.driveMotorPosition = mainDriveMotor.getEncoder().getPosition();
        inputs.driveMotorVelocity = getVelocity();
        inputs.driveMotorVelocitySetpoint = driveMotorVelocitySetpoint;

        inputs.angleMotorPosition = angleMotor.getEncoder().getPosition();
        angleMotorPosition = inputs.angleMotorPosition;
        inputs.angleMotorVelocity = angleMotor.getEncoder().getVelocity();

        inputs.angle = getAngle();
        currentAngle = inputs.angle;

        inputs.angleSetpoint = angleSetpoint;

        inputs.moduleDistance = getModulePosition().distanceMeters;
    }

    @Override
    public double getAngle() {
        return AngleUtil.normalize(ticksPerRad.toUnits(angleMotor.getEncoder().getPosition()));
    }

    @Override
    public void setAngle(double angle) {
        angleSetpoint = AngleUtil.normalize(angle);
        Rotation2d error = new Rotation2d(angle).minus(new Rotation2d(currentAngle));
        angleMotor.set(anglePID.calculate(angleMotorPosition + ticksPerRad.toTicks(error.getRadians())));
    }

    @Override
    public double getVelocity() {
        return ticksPerMeter.toVelocity(mainDriveMotor.getEncoder().getVelocity());
    }

    @Override
    public void setVelocity(double velocity) {
        var angleError = new Rotation2d(angleSetpoint).minus(new Rotation2d(currentAngle));
        velocity *= angleError.getCos();
        driveMotorVelocitySetpoint = velocity;
        mainDriveMotor.set(drivePID.calculate(ticksPerMeter.toTicks100ms(velocity)));
    }

    @Override
    public SwerveModulePosition getModulePosition() {
        return new SwerveModulePosition(
                ticksPerMeter.toUnits(mainDriveMotor.getEncoder().getPosition()),
                new Rotation2d(getAngle())
        );
    }

    @Override
    public void updateOffset(double offset) {
        angleMotor.getEncoder().setPosition(
                ((encoder.getAbsolutePosition() - offset) * 42) / SwerveConstants.ANGLE_REDUCTION);
    }

    @Override
    public void stop() {
        mainDriveMotor.set(0);
        angleMotor.set(0);
    }

    @Override
    public boolean encoderConnected() {
        return encoder.isConnected();
    }
}
