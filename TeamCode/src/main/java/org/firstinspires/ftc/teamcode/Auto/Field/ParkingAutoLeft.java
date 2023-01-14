package org.firstinspires.ftc.teamcode.Auto.Field;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Auto.Detection.ObjectDetector;
import org.firstinspires.ftc.teamcode.Robot.Bot;
import org.firstinspires.ftc.teamcode.Robot.Variables;


@Autonomous(name= "Left Parking Auto")
public class ParkingAutoLeft extends LinearOpMode {

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
        ObjectDetector.POSITIONS position = detector.getDecision(this);
        pos = position;
        telemetry.addData("position ", detector.getDecision(this));

        if (ACTI()) {

            ACTII();
        }
        ACTIII();
    }

    boolean ACTI() {
        Bot.Claw.setTargetPosition(var.claw_cone);
        sleep(1);
        Bot.Claw.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Bot.Claw.setPower(-1);
        sleep(550);
        Bot.strafeDrive(75, .4, this);
        sleep(5);
        Bot.driveStraight(125, .5, this);
        sleep(5);
        Bot.distance.getDistance(DistanceUnit.CM);
        int i = 80;
        while (opModeIsActive() && Bot.distance.getDistance(DistanceUnit.CM) < 40 && i > 0) {
            sleep(50);
            i--;
        }
        if (i == 0) {
            Bot.SensorStrafeDrive(-50, 0.2, this);
            Bot.strafeDrive(-12, .3, this);
            return false;
        }
        Bot.SensorStrafeDrive(-50, .2, this);
        Bot.strafeDrive(-15, 0.3, this);
        return true;
    }
    void ACTII(){
        Bot.Lift.setTargetPosition(var.Lvl_Tall);
        sleep(1);
        Bot.Lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Bot.Lift.setPower(1);
        sleep(-var.Lvl_Tall);
        Bot.driveStraight(14, .3, this);
        Bot.Lift.setTargetPosition(var.Lvl_Tall + 600);
        sleep(75);
        //Bot.strafeDrive(3,.5,this);

        Bot.Claw.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Bot.Claw.setTargetPosition(var.claw_zero);
        sleep(1);
        Bot.Claw.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Bot.Claw.setPower(1);
        sleep(200);
        Bot.driveStraight(-15, .3, this);
        sleep(1);
        Bot.Claw.setTargetPosition(var.claw_cone);
        sleep(1200);
        Bot.Lift.setTargetPosition(var.Lvl_Ground);
        //sleep(-var.Lvl_Tall);
        Bot.Lift.setPower(0);
    }

    void ACTIII(){
        switch (pos) {
            case POS1:
                Bot.strafeDrive(-98,.5,this);
                break;
            case POS2:
                Bot.strafeDrive(-40,.5,this);
                break;
            case POS3:
                Bot.strafeDrive(35,.5,this);
        }
    }
}
