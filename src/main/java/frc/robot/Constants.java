package frc.robot;


import com.ctre.phoenix.motorcontrol.TalonFXInvertType;

public final class Constants {
    public static final int TALON_TIMEOUT = 10;
    public static final TalonFXInvertType CLOCKWISE = TalonFXInvertType.Clockwise;
    public static final TalonFXInvertType COUNTER_CLOCKWISE = TalonFXInvertType.CounterClockwise;
    public static final double NOMINAL_VOLTAGE = 10.0;
    public static final double LOOP_PERIOD = 0.02; // [s]
    public static final double FIELD_LENGTH = 16.54; //2023 game
    public static final double FIELD_WIDTH = 8.02; //2023 game
}
