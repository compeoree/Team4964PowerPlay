package org.firstinspires.ftc.teamcode.Auto.Field;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Auto.Detection.ObjectDetector;
import org.firstinspires.ftc.teamcode.Robot.Bot;
import org.firstinspires.ftc.teamcode.Robot.Variables;


@Autonomous(name= "RouxParkingAuto")
public class ParkingAutoRogue extends LinearOpMode {

    Bot robot = new Bot();
    Variables var = new Variables();

    @Override
    public void runOpMode() throws InterruptedException {
        ObjectDetector detector = new ObjectDetector(this, false,false);

        robot.init(hardwareMap, this);

        robot.bLeftDT.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.tLeftDT.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.bRightDT.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.tRightDT.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        waitForStart();

        ObjectDetector.POSITIONS position = detector.getDecision();
        robot.strafeDrive(40, 0, 0.7, this);
        robot.strafeDrive(0,70, 0.7, this);
        robot.strafeDrive(39, 0, 0.7, this);
        // getting into position to drop cone
        robot.Lift.setTargetPosition(var.Lvl_Tall);
        robot.strafeDrive(0, 4, 0.7, this);
        robot.Claw.setPosition(1);
        robot.strafeDrive(0, -4, 0.7, this);
        robot.Lift.setTargetPosition(var.Lvl_Ground);
        robot.Claw.setPosition(0);
        robot.strafeDrive(-30, 0, 0.7, this);
        robot.strafeDrive(0, 65, 0.7, this);
        // make the decision
        switch (position) {
            case POS1:
                break;
            case POS2:
                robot.strafeDrive(-55, 0, 0.7, this);
                break;
            case POS3:
                robot.strafeDrive(-112, 0, 0.7, this);
        }

    }
}
