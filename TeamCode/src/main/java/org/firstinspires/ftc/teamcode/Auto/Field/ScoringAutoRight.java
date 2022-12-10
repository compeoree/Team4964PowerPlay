package org.firstinspires.ftc.teamcode.Auto.Field;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Auto.Detection.ObjectDetector;
import org.firstinspires.ftc.teamcode.Robot.Bot;
import org.firstinspires.ftc.teamcode.Robot.Variables;


@Autonomous(name= "Right Scoring Auto")
public class ScoringAutoRight extends LinearOpMode {

    Bot robot = new Bot();
    Variables var = new Variables();
    ObjectDetector.POSITIONS pos;

    @Override
    public void runOpMode() throws InterruptedException {
        ObjectDetector detector = new ObjectDetector(this, true,false);

        robot.init(hardwareMap, this);

        Bot.bLeftDT.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Bot.tLeftDT.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Bot.bRightDT.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Bot.tRightDT.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Bot.Lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Bot.Claw.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        waitForStart();

        // camera decision




        //robot.strafeDrive(-40, 0, 0.7, this);
        //robot.strafeDrive(0,70, 0.7, this);
       // robot.strafeDrive(-39, 0, 0.7, this);


        ACTI();

        //ACTII();

        //ACTIII();


//
//
    //    // getting into position to drop cone
    //    //robot.strafeDrive(0, 4, 0.7, this);
    //    Bot.driveStraight(.7, 4,4,4,4,this);
//
    //    Bot.Claw.setTargetPosition(var.claw_open);
    //    //robot.strafeDrive(0, -4, 0.7, this);
    //    Bot.driveStraight(.7, -4,-4,-4,-4,this);
    //    Bot.Lift.setTargetPosition(var.Lvl_Ground);
    //    Bot.Claw.setTargetPosition(var.claw_zero);
    //    //robot.strafeDrive(30, 0, 0.7, this);
    //    //robot.strafeDrive(0, 65, 0.7, this);
    //    Bot.strafeDrive(30,.7, this);
    //    Bot.driveStraight(.7, 65,65,65,65,this);
//
//
    //    // make the decision
    //    switch (position) {
    //        case POS1:
    //         break;
    //         case POS2:
    //             //robot.strafeDrive(55, 0, 0.7, this);
    //             Bot.strafeDrive(55,.7, this);
//
    //             break;
    //             case POS3:
    //                 //robot.strafeDrive(112, 0, 0.7, this);
    //                 Bot.strafeDrive(112,.7, this);
//
    //    }
    }

    void ACTI(){
        Bot.Claw.setTargetPosition(var.claw_cone);
        sleep(1);
        Bot.Claw.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Bot.Claw.setPower(1);
        sleep(55);
        Bot.driveStraight(65,.5,this);
        sleep(5);
        Bot.Claw.setTargetPosition(var.claw_zero);
        sleep(550);
        Bot.driveStraight(5,.9,this);
        sleep(5);
        Bot.driveStraight(-5,.9,this);
        sleep(5);
    }

    void ACTII(){
        Bot.strafeDrive(-115, .6, this);
        sleep(5);
        Bot.driveStraight(5,.9,this);
        sleep(5);
        Bot.Lift.setTargetPosition(var.Lvl_Short + 400);
        Bot.Lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Bot.Lift.setPower(1);
        sleep(550);
        Bot.Claw.setTargetPosition(var.claw_cone);
        sleep(55);
    }

    void ACTIII(){
        Bot.driveStraight(125, .5, this);
        sleep(5);
        Bot.strafeDrive(-22, .8, this);
        sleep(5);
    }

    void ACTIV(){
        Bot.Lift.setTargetPosition(var.Lvl_Tall);
        sleep(-var.Lvl_Tall);
        Bot.driveStraight(16, .3, this);
        Bot.Lift.setTargetPosition(var.Lvl_Tall + 200);
        sleep(75);
        Bot.strafeDrive(3,.5,this);
        Bot.Claw.setTargetPosition(var.claw_zero);
        sleep(2000);
        Bot.driveStraight(-20, .3, this);
        sleep(1);
        Bot.Claw.setTargetPosition(var.claw_cone);
        sleep(600);
        Bot.Lift.setTargetPosition(var.Lvl_Ground);
        sleep(-var.Lvl_Tall);
        Bot.Lift.setPower(0);
    }

    void ACTV(){
        Bot.strafeDrive(137, .5, this);
        sleep(5);
        Bot.driveStraight(125, .6, this);
        sleep(5);
    }
}
