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
    static final double     COUNTS_PER_MOTOR_REV    = 550;
    static final double     WHEEL_DIAMETER_INCHES   = 4.125;
    //16.7 is encoder counts per centimeter
    public static final double conversion = 16.7;
}