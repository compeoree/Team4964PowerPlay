package org.firstinspires.ftc.teamcode.Auto.Field;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Auto.Detection.ObjectDetector;
import org.firstinspires.ftc.teamcode.Robot.Bot;


@Autonomous(name= "ParkingAuto")
public class ParkingAuto extends LinearOpMode {

    Bot robot = new Bot();

    @Override
    public void runOpMode() throws InterruptedException {
        ObjectDetector detector = new ObjectDetector(this, false,false);

        robot.init(hardwareMap, this);

        robot.bLeftDT.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.tLeftDT.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.bRightDT.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.tRightDT.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        waitForStart();

        //move forward and scan cone
        //strafe left
        //move forward
        //strafe right
        //put cone on pole
        //move to the correct position indicated by cone scanned
    }
}
