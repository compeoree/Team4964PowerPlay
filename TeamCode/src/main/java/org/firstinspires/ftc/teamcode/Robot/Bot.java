package org.firstinspires.ftc.teamcode.Robot;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Bot {
    public DcMotor tLeftDT  = null;
    public DcMotor bLeftDT  = null;
    public DcMotor tRightDT = null;
    public DcMotor bRightDT = null;
    public DcMotor Lift     = null;
    public Servo   Claw     = null;
    public ModernRoboticsI2cGyro gyro = null;

    public void init(HardwareMap ahwMap, OpMode opMode) {
        hwMap = ahwMap;

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

        tLeftDT.setDirection(DcMotor.Direction.REVERSE);
        bLeftDT.setDirection(DcMotor.Direction.FORWARD);

        bLeftDT.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        tLeftDT.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bLeftDT.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        tLeftDT.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        tLeftDT.setPower(0);
        bLeftDT.setPower(0);
        tRightDT.setPower(0);
        bRightDT.setPower(0);
        Lift.setPower(0);
        //fix later\\
        Claw.setPosition(0.4);

        opMode.telemetry.addLine("Initialization Complete");
        opMode.telemetry.update();
    }

}
