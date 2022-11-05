package org.firstinspires.ftc.teamcode.Robot;

public class Variables  {
    public int Lvl_Ground = 0,
            Lvl_Short  = -1400,
            Lvl_Mid    = -2400,
            Lvl_Tall   = -3200;
    public boolean slo,
            btnlock = false;
    public int claw_zero = 5,
            claw_open = -28,
            claw_cone = -13;
    static final double     COUNTS_PER_MOTOR_REV    = 19+(1/3);
    static final double     DRIVE_GEAR_REDUCTION    = 40;
    static final double     WHEEL_DIAMETER_INCHES   = 4.125;
    static final double     EMPIRICAL_MULTIPLIER    = (30.0 / 17);
    static final double     COUNTS_PER_INCH  = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION * EMPIRICAL_MULTIPLIER)
            / (WHEEL_DIAMETER_INCHES * Math.PI);
    public static final double conversion = COUNTS_PER_INCH * 2.54;
}