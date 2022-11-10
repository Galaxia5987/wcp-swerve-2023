package frc.robot.subsystems.drivetrain;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Constants;
import frc.robot.subsystems.LoggedSubsystem;
import frc.robot.utils.motors.PIDTalon;
import frc.robot.utils.valuetuner.WebConstant;

public class SwerveModule extends LoggedSubsystem {
    private final WPI_TalonFX driveMotor;
    private final PIDTalon angleMotor;
    private final SwerveDrive.Module number;
    private final int offset;

    private final WebConstant webKp;
    private final WebConstant webKi;
    private final WebConstant webKd;
    private final WebConstant webKf;

    private final SwerveModuleLogInputs inputs;

    public SwerveModule(SwerveDrive.Module number, int driveMotorPort, int angleMotorPort, int offset, boolean driveInverted,
                        boolean angleInverted, boolean angleSensorPhase, double[] motionMagicConfigs) throws Exception {
        super(SwerveModuleLogInputs.getInstance(number.number));
        if (motionMagicConfigs.length != 10) {
            throw new Exception("Improper motion magic config!");
        }
        inputs = SwerveModuleLogInputs.getInstance(number.number);
        driveMotor = new WPI_TalonFX(driveMotorPort);
        angleMotor = new PIDTalon(angleMotorPort);
        this.number = number;
        this.offset = offset;

        driveMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, Constants.TALON_TIMEOUT);
        driveMotor.setInverted(driveInverted);
        driveMotor.setNeutralMode(NeutralMode.Brake);
        driveMotor.selectProfileSlot(1, 0);

        angleMotor.configSelectedFeedbackSensor(FeedbackDevice.Analog, 0, Constants.TALON_TIMEOUT);
        angleMotor.configFeedbackNotContinuous(false, Constants.TALON_TIMEOUT);
        angleMotor.setInverted(angleInverted);
        angleMotor.setSensorPhase(angleSensorPhase);

        configMotionMagic(motionMagicConfigs);

        angleMotor.setNeutralMode(NeutralMode.Brake);
        angleMotor.selectProfileSlot(0, 0);

        webKp = WebConstant.of(getSubsystemName(), "Angle kP", motionMagicConfigs[MotionMagicConfig.Kp.index]);
        webKi = WebConstant.of(getSubsystemName(), "Angle kI", motionMagicConfigs[MotionMagicConfig.Ki.index]);
        webKd = WebConstant.of(getSubsystemName(), "Angle kD", motionMagicConfigs[MotionMagicConfig.Kd.index]);
        webKf = WebConstant.of(getSubsystemName(), "Angle kF", motionMagicConfigs[MotionMagicConfig.Kf.index]);
    }

    @Override
    public void updateInputs() {
        inputs.aPosition = angleMotor.getSelectedSensorPosition();
        inputs.aAngle = toRotation2d(inputs.aPosition - offset);
        inputs.aCurrent = angleMotor.getSupplyCurrent();

        inputs.dVelocity = driveMotor.get();
        inputs.dCurrent = driveMotor.getSupplyCurrent();
    }

    @Override
    public String getSubsystemName() {
        return "SwerveModule_" + number.name();
    }

    public void configMotionMagic(double[] motionMagicConfigs) {
        angleMotor.config_kP(0, motionMagicConfigs[MotionMagicConfig.Kp.index], Constants.TALON_TIMEOUT);
        angleMotor.config_kI(0, motionMagicConfigs[MotionMagicConfig.Ki.index], Constants.TALON_TIMEOUT);
        angleMotor.config_kD(0, motionMagicConfigs[MotionMagicConfig.Kd.index], Constants.TALON_TIMEOUT);
        angleMotor.config_kF(0, motionMagicConfigs[MotionMagicConfig.Kf.index], Constants.TALON_TIMEOUT);
        angleMotor.configMotionSCurveStrength((int) motionMagicConfigs[MotionMagicConfig.SCurveStrength.index]);
        angleMotor.configMotionCruiseVelocity(motionMagicConfigs[MotionMagicConfig.CruiseVelocity.index]);
        angleMotor.configMotionAcceleration(motionMagicConfigs[MotionMagicConfig.MaxAcceleration.index]);
        angleMotor.configAllowableClosedloopError(0, motionMagicConfigs[MotionMagicConfig.ClosedLoopError.index]);
        angleMotor.configMaxIntegralAccumulator(0, motionMagicConfigs[MotionMagicConfig.MaxIntegralAccumulator.index]);
        angleMotor.configClosedLoopPeakOutput(0, motionMagicConfigs[MotionMagicConfig.ClosedLoopPeakOutput.index]);
    }

    public Rotation2d toRotation2d(double ticks) {
        return new Rotation2d(ticks % Constants.TICKS_PER_ROTATION_ANGLE / Constants.TICKS_PER_ROTATION_ANGLE * 2 * Math.PI);
    }

    public int toTicks(Rotation2d angle) {
        return (int) (angle.getRadians() / (2 * Math.PI) * Constants.TICKS_PER_ROTATION_ANGLE);
    }

    public Rotation2d getAngle() {
        return inputs.aAngle;
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(inputs.dVelocity, getAngle());
    }

    public void set(double speed, Rotation2d angle) {
        SwerveModuleState optimized = SwerveModuleState.optimize(new SwerveModuleState(speed, angle), getAngle());
        speed = optimized.speedMetersPerSecond;
        angle = optimized.angle;

        driveMotor.set(ControlMode.PercentOutput, speed);

        int error = toTicks(angle.minus(toRotation2d(inputs.aPosition))) + offset;
        angleMotor.set(ControlMode.MotionMagic, inputs.aPosition + error);
    }

    @Override
    public void periodic() {
        angleMotor.updatePID(0, webKp.get(), webKi.get(), webKd.get(), webKf.get());
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
