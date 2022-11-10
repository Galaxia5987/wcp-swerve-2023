package frc.robot.subsystems.drivetrain;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.LinearQuadraticRegulator;
import edu.wpi.first.math.estimator.KalmanFilter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.LinearSystemLoop;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;
import frc.robot.subsystems.LoggedSubsystem;
import frc.robot.subsystems.drivetrain.config.SwerveModuleConfigBase;
import frc.robot.utils.units.UnitModel;
import frc.robot.utils.units.Units;

/**
 * This subsystem represents a single Swerve module and is responsible for the basic operation of a module, such as,
 * rotating to a specific angle, driving at a specific rate, and other convenience features for tuning the coefficients.
 */
public class SwerveModule extends LoggedSubsystem {
    private final WPI_TalonFX driveMotor;
    private final WPI_TalonSRX angleMotor;
    private final UnitModel driveUnitModel;
    private final UnitModel angleUnitModel;

    private final SwerveModuleConfigBase config;
    private final SwerveModuleLogInputs inputs;
    private LinearSystemLoop<N1, N1, N1> stateSpace;
    private double currentTime, lastTime;
    private double lastJ;

    public SwerveModule(SwerveModuleConfigBase config) {
        super(SwerveModuleLogInputs.getInstance(config.wheel()));
        this.config = config;
        inputs = SwerveModuleLogInputs.getInstance(config.wheel());
        driveMotor = new WPI_TalonFX(config.driveMotorPort());
        angleMotor = new WPI_TalonSRX(config.angleMotorPort());
        driveUnitModel = new UnitModel(Constants.SwerveDrive.DRIVE_MOTOR_TICKS_PER_METER);
        angleUnitModel = new UnitModel(Constants.SwerveDrive.ANGLE_MOTOR_TICKS_PER_RADIAN);
        stateSpace = constructVelocityLinearSystem(config.j());
        lastJ = config.j();

        // configure feedback sensors
        angleMotor.configSelectedFeedbackSensor(FeedbackDevice.Analog, 0, Constants.TALON_TIMEOUT);
        angleMotor.configFeedbackNotContinuous(false, Constants.TALON_TIMEOUT);

        driveMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, Constants.TALON_TIMEOUT);

        angleMotor.setNeutralMode(NeutralMode.Brake);
        driveMotor.setNeutralMode(NeutralMode.Brake);

        // inversions
        driveMotor.setInverted(config.driveMotorInverted());
        angleMotor.setInverted(config.angleMotorInverted());

        angleMotor.setSensorPhase(config.angleMotorSensorPhase());

        // Set amperage limits
        SupplyCurrentLimitConfiguration currLimitConfig = new SupplyCurrentLimitConfiguration(
                true,
                Constants.SwerveDrive.MAX_CURRENT,
                Constants.SwerveModule.TRIGGER_THRESHOLD_CURRENT,
                Constants.SwerveModule.TRIGGER_THRESHOLD_TIME
        );

        driveMotor.configSupplyCurrentLimit(currLimitConfig);

        angleMotor.configSupplyCurrentLimit(currLimitConfig);
        angleMotor.enableCurrentLimit(true);

        // set PIDF - angle motor
        configPID(config.angleKp(), config.angleKi(), config.angleKd(), config.angleKf());
        angleMotor.config_IntegralZone(0, 5);
        angleMotor.configAllowableClosedloopError(0, angleUnitModel.toTicks(Constants.SwerveDrive.ALLOWABLE_ANGLE_ERROR));
        angleMotor.configMotionAcceleration(Constants.SwerveDrive.ANGLE_MOTION_ACCELERATION);
        angleMotor.configMotionCruiseVelocity(Constants.SwerveDrive.ANGLE_CRUISE_VELOCITY);

        angleMotor.configMotionSCurveStrength(Constants.SwerveDrive.ANGLE_CURVE_STRENGTH);

        // set voltage compensation and saturation
        angleMotor.enableVoltageCompensation(true);
        angleMotor.configVoltageCompSaturation(Constants.NOMINAL_VOLTAGE);

        angleMotor.selectProfileSlot(0, 0);
        driveMotor.selectProfileSlot(1, 0);
        driveMotor.configIntegratedSensorInitializationStrategy(SensorInitializationStrategy.BootToZero);

        selectTuneDownMode(true);
    }

    /**
     * Initialize the linear system to the default values in order to use the
     * state-space model controlling the velocity of the drive motor.
     *
     * @return an object that represents the model to reach the velocity at the best rate.
     */
    private LinearSystemLoop<N1, N1, N1> constructVelocityLinearSystem(double j) {
        if (j <= 0) throw new IllegalStateException("j must have non-zero value");
        // https://file.tavsys.net/control/controls-engineering-in-frc.pdf Page 76
        LinearSystem<N1, N1, N1> stateSpace = LinearSystemId.createFlywheelSystem(DCMotor.getFalcon500(1),
                j, Constants.SwerveDrive.GEAR_RATIO_DRIVE_MOTOR);
        KalmanFilter<N1, N1, N1> kalman = new KalmanFilter<>(Nat.N1(), Nat.N1(), stateSpace,
                VecBuilder.fill(Constants.SwerveDrive.MODEL_TOLERANCE),
                VecBuilder.fill(Constants.SwerveDrive.ENCODER_TOLERANCE),
                Constants.LOOP_PERIOD
        );
        LinearQuadraticRegulator<N1, N1, N1> lqr = new LinearQuadraticRegulator<>(stateSpace,
                VecBuilder.fill(Constants.SwerveDrive.VELOCITY_TOLERANCE),
                VecBuilder.fill(Constants.SwerveDrive.COST_LQR),
                Constants.LOOP_PERIOD // time between loops, DON'T CHANGE
        );
        lqr.latencyCompensate(stateSpace, Constants.LOOP_PERIOD, Constants.TALON_TIMEOUT * 0.001);

        return new LinearSystemLoop<>(stateSpace, lqr, kalman, Constants.NOMINAL_VOLTAGE, Constants.LOOP_PERIOD);
    }


    /**
     * Gets the velocity of the drive motor.
     *
     * @return the velocity of the wheel. [m/s]
     */
    public double getVelocity() {
        return inputs.dVelocity;
    }

    /**
     * Sets the velocity of the drive motor.
     *
     * @param velocity the velocity of the module. [m/s]
     */
    public void setVelocity(double velocity) {
        if (DriverStation.isAutonomous()) {
            double timeInterval = currentTime - lastTime;
            double targetVelocity = Units.metersPerSecondToRps(velocity, Constants.SwerveDrive.WHEEL_RADIUS);
            double currentVelocity = Units.metersPerSecondToRps(getVelocity(), Constants.SwerveDrive.WHEEL_RADIUS);

            stateSpace.setNextR(VecBuilder.fill(targetVelocity)); // r = reference (setpoint)
            stateSpace.correct(VecBuilder.fill(currentVelocity));
            stateSpace.predict(timeInterval);

            // u = input, the needed input in order to come to the next state optimally
            driveMotor.setVoltage(stateSpace.getU(0));
        } else {
            driveMotor.set(velocity);
        }
    }

    /**
     * Gets the angle that the module is pointing.
     *
     * @return the angle of the module. [rad]
     */
    public Rotation2d getAngle() {
        return Rotation2d.fromDegrees(inputs.aAngle);
    }

    /**
     * Sets the angle of the module, with consideration of the shortest path to the target angle.
     *
     * @param angle the target angle.
     */
    public void setAngle(Rotation2d angle) {
        var currentAngle = getAngle();
        var error = angle.minus(currentAngle);

        angleMotor.set(ControlMode.MotionMagic, angleMotor.getSelectedSensorPosition() + angleUnitModel.toTicks(error.getRadians()));
    }

    /**
     * Gets the state of the module.
     *
     * @return the current state of the module.
     */
    public SwerveModuleState getState() {
        return new SwerveModuleState(getVelocity(), getAngle());
    }

    /**
     * Update the state of the module.
     *
     * @param state the desired state.
     */
    public void setState(SwerveModuleState state) {
        setVelocity(state.speedMetersPerSecond);
        setAngle(state.angle);
    }

    /**
     * Set the power of angle motor for this module for testing purposes.
     *
     * @param power percent power to give to the angel motor. [%]
     */
    public void setPower(double power) {
        angleMotor.set(power);
    }

    /**
     * Stops the drive motor from moving.
     */
    public void stopDriveMotor() {
        driveMotor.stopMotor();
    }

    /**
     * Stops the angle motor from moving.
     */
    public void stopAngleMotor() {
        angleMotor.stopMotor();
    }

    /**
     * Runs the motor at full power (only used for testing purposes).
     */
    public void setMaxOutput() {
        driveMotor.set(1);
        angleMotor.set(1);
    }

    /**
     * Gets the wheel number.
     *
     * @return the wheel number.
     */
    public int getWheel() {
        return config.wheel();
    }

    /**
     * Configures the pid coefficients of the angle motor.
     *
     * @param kp proportional coefficients.
     * @param ki integral coefficients.
     * @param kd derivative coefficients.
     * @param kf feedforward coefficients.
     */
    private void configPID(double kp, double ki, double kd, double kf) {
        angleMotor.config_kP(0, kp, Constants.TALON_TIMEOUT);
        angleMotor.config_kI(0, ki, Constants.TALON_TIMEOUT);
        angleMotor.config_kD(0, kd, Constants.TALON_TIMEOUT);
        angleMotor.config_kF(0, kf, Constants.TALON_TIMEOUT);
    }

    private void selectTuneDownMode(boolean isUserDefined) {
        double ramp;
        if (isUserDefined) {
            ramp = 0;
            angleMotor.configOpenloopRamp(0, Constants.TALON_TIMEOUT);
            angleMotor.configClosedloopRamp(0, Constants.TALON_TIMEOUT);
        } else {
            ramp = Constants.SwerveModule.RAMP_RATE;
        }
        driveMotor.configOpenloopRamp(ramp, Constants.TALON_TIMEOUT);
        driveMotor.configClosedloopRamp(ramp, Constants.TALON_TIMEOUT);

    }

    @Override
    public void updateInputs() {
        inputs.aVelocity = angleUnitModel.toVelocity(angleMotor.getSelectedSensorVelocity());
        inputs.aPosition = angleMotor.getSelectedSensorPosition();
        inputs.aAngle = new Rotation2d(Math.IEEEremainder(angleUnitModel.toUnits(angleMotor.getSelectedSensorPosition() - config.zeroPosition()), 2 * Math.PI)).getDegrees();
        inputs.dCurrent = angleMotor.getStatorCurrent();

        inputs.dVelocity = driveUnitModel.toVelocity(driveMotor.getSelectedSensorVelocity());
        inputs.dCurrent = driveMotor.getSupplyCurrent();
    }

    @Override
    public String getSubsystemName() {
        return "SwerveModule" + config.wheel();
    }

    @Override
    public void periodic() {
        if (config.debug()) {
            configPID(config.angleKp(), config.angleKi(), config.angleKd(), config.angleKf());
            if (config.j() != lastJ) {
                stateSpace = constructVelocityLinearSystem(config.j());
                stateSpace.reset(VecBuilder.fill(Units.metersPerSecondToRps(getVelocity(), Constants.SwerveDrive.WHEEL_RADIUS)));
                lastJ = config.j();
            }
        }
        // There is no need for kalman, so we technically disable him that way
        stateSpace.getObserver().reset();
        lastTime = currentTime;
        currentTime = Timer.getFPGATimestamp();
    }
}
