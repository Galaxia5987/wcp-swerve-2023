package frc.robot.subsystems.drivetrain;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.LoggedSubsystem;
import frc.robot.utils.motors.PIDTalon;

import static frc.robot.Constants.*;

public class SwerveModule extends LoggedSubsystem<SwerveModuleLogInputs> {
    private final WPI_TalonFX driveMotor;
    private final PIDTalon angleMotor;
    private final DutyCycleEncoder encoder;
    private final int offset;
    private final SwerveDrive.Module number;
    private final double[] motionMagicConfigs;
    private boolean initializedOffset = false;

    public SwerveModule(SwerveDrive.Module number, int driveMotorPort, int angleMotorPort, int encoderPort, int offset, boolean driveInverted,
                        boolean angleInverted, boolean angleSensorPhase, double[] motionMagicConfigs) {
        super(new SwerveModuleLogInputs());
        this.number = number;
        this.offset = offset;
        this.motionMagicConfigs = motionMagicConfigs;
        driveMotor = new WPI_TalonFX(driveMotorPort);
        angleMotor = new PIDTalon(angleMotorPort);

        driveMotor.configFactoryDefault();
        angleMotor.configFactoryDefault();

        driveMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, TALON_TIMEOUT);
        driveMotor.setInverted(driveInverted);
        driveMotor.setNeutralMode(NeutralMode.Brake);
        driveMotor.selectProfileSlot(1, 0);
        driveMotor.configNeutralDeadband(0.1);

        angleMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, TALON_TIMEOUT);
        angleMotor.configFeedbackNotContinuous(false, TALON_TIMEOUT);
        angleMotor.setInverted(angleInverted);
        angleMotor.setSensorPhase(angleSensorPhase);
        configMotionMagic(motionMagicConfigs);

        angleMotor.setNeutralMode(NeutralMode.Brake);
        angleMotor.selectProfileSlot(0, 0);

        encoder = new DutyCycleEncoder(encoderPort);

        SmartDashboard.putNumber(number.name() + "_kP", motionMagicConfigs[MotionMagicConfig.Kp.index]);
        SmartDashboard.putNumber(number.name() + "_kI", motionMagicConfigs[MotionMagicConfig.Ki.index]);
        SmartDashboard.putNumber(number.name() + "_kD", motionMagicConfigs[MotionMagicConfig.Kd.index]);
        SmartDashboard.putNumber(number.name() + "_kF", motionMagicConfigs[MotionMagicConfig.Kf.index]);
        SmartDashboard.putNumber(number.name() + "_sCurveStrength", motionMagicConfigs[MotionMagicConfig.SCurveStrength.index]);
        SmartDashboard.putNumber(number.name() + "_cruiseVelocity", motionMagicConfigs[MotionMagicConfig.CruiseVelocity.index]);
        SmartDashboard.putNumber(number.name() + "_maxAcceleration", motionMagicConfigs[MotionMagicConfig.MaxAcceleration.index]);
        SmartDashboard.putNumber(number.name() + "_allowableClosedLoopError", motionMagicConfigs[MotionMagicConfig.ClosedLoopError.index]);
        SmartDashboard.putNumber(number.name() + "_maxIntegralAccumulator", motionMagicConfigs[MotionMagicConfig.MaxIntegralAccumulator.index]);
        SmartDashboard.putNumber(number.name() + "_closedLoopPeakOutput", motionMagicConfigs[MotionMagicConfig.ClosedLoopPeakOutput.index]);
    }

    public void configMotionMagic(double[] motionMagicConfigs) {
        angleMotor.config_kP(0, motionMagicConfigs[MotionMagicConfig.Kp.index], TALON_TIMEOUT);
        angleMotor.config_kI(0, motionMagicConfigs[MotionMagicConfig.Ki.index], TALON_TIMEOUT);
        angleMotor.config_kD(0, motionMagicConfigs[MotionMagicConfig.Kd.index], TALON_TIMEOUT);
        angleMotor.config_kF(0, motionMagicConfigs[MotionMagicConfig.Kf.index], TALON_TIMEOUT);
        angleMotor.configMotionSCurveStrength((int) motionMagicConfigs[MotionMagicConfig.SCurveStrength.index]);
        angleMotor.configMotionCruiseVelocity(motionMagicConfigs[MotionMagicConfig.CruiseVelocity.index]);
        angleMotor.configMotionAcceleration(motionMagicConfigs[MotionMagicConfig.MaxAcceleration.index]);
        angleMotor.configAllowableClosedloopError(0, motionMagicConfigs[MotionMagicConfig.ClosedLoopError.index]);
        angleMotor.configMaxIntegralAccumulator(0, motionMagicConfigs[MotionMagicConfig.MaxIntegralAccumulator.index]);
        angleMotor.configClosedLoopPeakOutput(0, motionMagicConfigs[MotionMagicConfig.ClosedLoopPeakOutput.index]);
    }

    public void set(double speed, Rotation2d angle) {
        SwerveModuleState optimized = SwerveModuleState.optimize(new SwerveModuleState(speed, angle), loggerInputs.aAngle);
        speed = optimized.speedMetersPerSecond;
        angle = optimized.angle;
        loggerInputs.aSetpoint = angle; // Setpoint angle of the wheel

        driveMotor.set(ControlMode.PercentOutput, speed);

        angleMotor.set(ControlMode.MotionMagic, toFalconTicks(loggerInputs.aSetpoint));
    }

    public Rotation2d toWheelAbsoluteAngle(double ticks) {
        return new Rotation2d(((ticks / TICKS_PER_ROTATION) * 2 * Math.PI * ANGLE_GEAR_RATIO) % (2 * Math.PI));
    }

    public int toFalconTicks(Rotation2d angle) {
        return (int) (((angle.getDegrees() / 360) * TICKS_PER_ROTATION) / ANGLE_GEAR_RATIO);
    }

    public int absoluteEncoderToAbsoluteFalcon(double encoder) {
        return (int) (encoder * TICKS_PER_ROTATION / ANGLE_GEAR_RATIO);
    }

    public Rotation2d getAngle() {
        return loggerInputs.aAngle;
    }

    public double getEncoderTicks() {
        return toFalconTicks(loggerInputs.encoderAngle);
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(loggerInputs.dVelocity, getAngle());
    }

    public void stop() {
        angleMotor.stopMotor();
        driveMotor.stopMotor();
    }

    @Override
    public void updateInputs() {
        double encoderRelativeAngle = absoluteEncoderToAbsoluteFalcon(encoder.getAbsolutePosition()) - offset;
        while (encoderRelativeAngle > TICKS_PER_ROTATION / 2) {
            encoderRelativeAngle -= TICKS_PER_ROTATION;
        }

        loggerInputs.aPosition = angleMotor.getSelectedSensorPosition();
        loggerInputs.aAngle = toWheelAbsoluteAngle(loggerInputs.aPosition); // Angle of the wheel
        loggerInputs.encoderAngle = toWheelAbsoluteAngle(absoluteEncoderToAbsoluteFalcon(encoder.getAbsolutePosition()));
        loggerInputs.encoderRelativeAngle = toWheelAbsoluteAngle(encoderRelativeAngle);
        loggerInputs.offsetAngle = toWheelAbsoluteAngle(offset);
        loggerInputs.aCurrent = angleMotor.getSupplyCurrent();

        loggerInputs.dVelocity = driveMotor.get();
        loggerInputs.dCurrent = driveMotor.getSupplyCurrent();
    }

    @Override
    public void periodic() {
        if (!initializedOffset && encoder.isConnected()) {
            double newPosition = absoluteEncoderToAbsoluteFalcon(encoder.getAbsolutePosition()) - offset;
            angleMotor.setSelectedSensorPosition(newPosition);
            initializedOffset = true;
        }

        if (SmartDashboard.getBoolean("Swerve Tune Motion Magic", false)) {
            angleMotor.updatePID(0,
                    SmartDashboard.getNumber(number.name() + "_kP", motionMagicConfigs[MotionMagicConfig.Kp.index]),
                    SmartDashboard.getNumber(number.name() + "_kI", motionMagicConfigs[MotionMagicConfig.Ki.index]),
                    SmartDashboard.getNumber(number.name() + "_kD", motionMagicConfigs[MotionMagicConfig.Kd.index]),
                    SmartDashboard.getNumber(number.name() + "_kF", motionMagicConfigs[MotionMagicConfig.Kf.index]));

            angleMotor.configMotionCruiseVelocity(
                    SmartDashboard.getNumber(number.name() + "_cruiseVelocity",
                            motionMagicConfigs[MotionMagicConfig.CruiseVelocity.index]));
            angleMotor.configMotionAcceleration(
                    SmartDashboard.getNumber(number.name() + "_maxAcceleration",
                            motionMagicConfigs[MotionMagicConfig.MaxAcceleration.index]));
        }
    }

    @Override
    public String getSubsystemName() {
        return "SwerveModule_" + number;
    }

    public enum MotionMagicConfig {
        Kp(0),
        Ki(1),
        Kd(2),
        Kf(3),
        SCurveStrength(4),
        CruiseVelocity(5),
        MaxAcceleration(6),
        ClosedLoopError(7),
        MaxIntegralAccumulator(8),
        ClosedLoopPeakOutput(9);

        public final int index;

        MotionMagicConfig(int index) {
            this.index = index;
        }
    }
}
