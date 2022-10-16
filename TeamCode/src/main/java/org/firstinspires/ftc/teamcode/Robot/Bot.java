package org.firstinspires.ftc.teamcode.Robot;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Bot {
    public static DcMotor tLeftDT  = null;
    public static DcMotor bLeftDT  = null;
    public static DcMotor tRightDT = null;
    public static DcMotor bRightDT = null;
    public static DcMotor Lift     = null;
    public static Servo   Claw     = null;
    public static ModernRoboticsI2cGyro gyro = null;



    public void init(HardwareMap ahwMap, OpMode opMode) {
        HardwareMap hwMap = ahwMap;

        //motor init
        tLeftDT   = hwMap.get(DcMotor.class, "tLeftDT");
        bLeftDT   = hwMap.get(DcMotor.class, "bLeftDT");
        tRightDT  = hwMap.get(DcMotor.class, "tRightDT");
        bRightDT  = hwMap.get(DcMotor.class, "bRightDT");
        Lift      = hwMap.get(DcMotor.class, "Lift"    );
        Claw      = hwMap.get(Servo.class,   "Claw"    );


        //gyro init
        gyro = hwMap.get(ModernRoboticsI2cGyro.class,"gyro");
        gyro.initialize();
        gyro.calibrate();
        while(gyro.isCalibrating());
        opMode.telemetry.addLine("Gyro Calibrated");
        opMode.telemetry.update();

        //motors init
        tRightDT.setDirection(DcMotor.Direction.REVERSE);
        bRightDT.setDirection(DcMotor.Direction.REVERSE);

        bLeftDT.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        tLeftDT.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bRightDT.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        tRightDT.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        bLeftDT.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        tLeftDT.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bRightDT.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        tRightDT.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        Lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        bLeftDT.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        tLeftDT.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bRightDT.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        tRightDT.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        tLeftDT.setPower(0);
        bLeftDT.setPower(0);
        tRightDT.setPower(0);
        bRightDT.setPower(0);
        Lift.setPower(0);
        //fix later
        Claw.setPosition(0.4);

        opMode.telemetry.addLine("Initialization Complete! ;) ");
        opMode.telemetry.update();


    }

    //driving using only Mecanum strafe
    public static void strafeDrive (float distanceX, float distanceY, double speed, LinearOpMode opMode)
    {
        double tLeftPower  = tLeftDT.getCurrentPosition()  + ( distanceX + 2 * (  distanceY * distanceX ));
        double bLeftPower  = bLeftDT.getCurrentPosition()  + ( distanceX + 2 * (  distanceY * distanceX ));
        double tRightPower = tRightDT.getCurrentPosition() + ( distanceX + 2 * ( -distanceY * distanceX ));
        double bRightPower = bRightDT.getCurrentPosition() + ( distanceX + 2 * ( -distanceY * distanceX ));

        tLeftDT.setPower(speed);
        bLeftDT.setPower(speed);
        tRightDT.setPower(speed);
        bRightDT.setPower(speed);

        speed = Range.clip(Math.abs(speed), 0.0, 1.0);
        tLeftDT.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        bLeftDT.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        tLeftDT.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        bLeftDT.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        tLeftDT.setTargetPosition( (int)tLeftPower);
        bLeftDT.setTargetPosition( (int)bLeftPower);
        tRightDT.setTargetPosition((int)tRightPower);
        bRightDT.setTargetPosition((int)bRightPower);

    }


}


