package frc.robot.subsystems.elevator;

import com.ctre.phoenix.motorcontrol.TalonFXInvertType;

public class ElevatorConstants {
    public static final TalonFXInvertType INVERT_TYPE = TalonFXInvertType.Clockwise; //TODO: check direction
    public static final double VOLT_COMP_SATURATION = 12;
    public static final int FALCON_TIMEOUT = 10;
    public static final double DRUM_RADIUS = 0.03; // Radius of the elevator drum. [m] //TODO: check with mechanics team
    public static final double TICKS_PER_METER = 2 * Math.PI * DRUM_RADIUS / 2048; // [tick]


    public static final double kP = 0;
    public static final double kI = 0;
    public static final double kD = 0;

}