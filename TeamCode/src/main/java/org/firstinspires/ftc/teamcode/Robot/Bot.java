package org.firstinspires.ftc.teamcode.Robot;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;



public class Bot {
    public static DcMotor tLeftDT  = null;
    public static DcMotor bLeftDT  = null;
    public static DcMotor tRightDT = null;
    public static DcMotor bRightDT = null;
    public static DcMotor Lift     = null;
    public static Servo   Claw     = null;


    public static final float conversion = 19.6666666666666666666666666f; // conversion of encoder rotations to centimetres
    public static final float tileConversion = 1; // conversion of encoder rotations to tiles !!EDIT!!

    public void init(HardwareMap ahwMap, OpMode opMode) {
        HardwareMap hwMap = ahwMap;

        //motor init
        tLeftDT   = hwMap.get(DcMotor.class, "FrontL");
        bLeftDT   = hwMap.get(DcMotor.class, "BackL");
        tRightDT  = hwMap.get(DcMotor.class, "FrontR");
        bRightDT  = hwMap.get(DcMotor.class, "BackR");
        Lift      = hwMap.get(DcMotor.class, "lift"    );
        Claw      = hwMap.get(Servo.class,   "claw"    );


        //gyro init

        opMode.telemetry.addLine("Gyro Calibrated");
        opMode.telemetry.update();

        //cam init




        //motors init
        tLeftDT.setDirection(DcMotor.Direction.REVERSE);
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
        // if it breaks do this https://github.com/AnishJag/FTCFreightFrenzy/blob/master/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/Base/MainBase.java
        if (opMode.opModeIsActive()) {
            double tLeftPower =   tLeftDT.getCurrentPosition() + conversion * (2 * (-distanceY) + distanceX);
            double bLeftPower =   bLeftDT.getCurrentPosition() + conversion * (2 * ( distanceY) + distanceX);
            double tRightPower = tRightDT.getCurrentPosition() + conversion * (2 * ( distanceY) + distanceX);
            double bRightPower = bRightDT.getCurrentPosition() + conversion * (2 * (-distanceY) + distanceX);

            tLeftDT.setPower(speed);
            bLeftDT.setPower(speed);
            tRightDT.setPower(speed);
            bRightDT.setPower(speed);

            tLeftDT.setTargetPosition((int) tLeftPower);
            bLeftDT.setTargetPosition((int) bLeftPower);
            tRightDT.setTargetPosition((int) tRightPower);
            bRightDT.setTargetPosition((int) bRightPower);

            speed = Range.clip(Math.abs(speed), 0.0, 1.0);
            tLeftDT.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            bLeftDT.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            tLeftDT.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            bLeftDT.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        }

    }

    public static void strafeDrive (float distanceX, float distanceY, double speed, boolean tiles, LinearOpMode opMode)
    {
        double tLeftPower  = tLeftDT.getCurrentPosition()  + tileConversion * (2 * (-distanceY) + distanceX);
        double bLeftPower  = bLeftDT.getCurrentPosition()  + tileConversion * (2 * ( distanceY) + distanceX);
        double tRightPower = tRightDT.getCurrentPosition() + tileConversion * (2 * ( distanceY) + distanceX);
        double bRightPower = bRightDT.getCurrentPosition() + tileConversion * (2 * (-distanceY) + distanceX);

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


