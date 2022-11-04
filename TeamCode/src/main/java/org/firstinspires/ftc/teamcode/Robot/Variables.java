package org.firstinspires.ftc.teamcode.Robot;

public class Variables  {
    public int Lvl_Ground = 0,
            Lvl_Short  = -2000,
            Lvl_Mid    = -2625,
            Lvl_Tall   = -3225;
    public boolean slo,
            btnlock = false;
    public int claw_zero = 1,
            claw_open = 40,
            claw_cone = 50;
    static final double     COUNTS_PER_MOTOR_REV    = 386.3;
    static final double     DRIVE_GEAR_REDUCTION    = 1.0;
    static final double     WHEEL_DIAMETER_INCHES   = 2.5;
    static final double     EMPIRICAL_MULTIPLIER    = (30.0 / 17);
    static final double     COUNTS_PER_INCH  = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION * EMPIRICAL_MULTIPLIER)
            / (WHEEL_DIAMETER_INCHES * 3.14159265);
    public static final double conversion = COUNTS_PER_INCH * 2.54;
}