package org.firstinspires.ftc.teamcode.Auto.Field;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Auto.Detection.ObjectDetector;
import org.firstinspires.ftc.teamcode.Robot.Bot;
import org.firstinspires.ftc.teamcode.Robot.Variables;

@Autonomous(name= "Left Parking Auto")
public class ParkingAutoLeft extends LinearOpMode {

    Bot robot = new Bot();
    Variables var = new Variables();

    @Override
    public void runOpMode() throws InterruptedException {
        ObjectDetector detector = new ObjectDetector(this, false, false);

        robot.init(hardwareMap, this);

        if (robot.bLeftDT != null)
            robot.bLeftDT.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        if (robot.tLeftDT != null)
            robot.tLeftDT.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        if (robot.bRightDT != null)
            robot.bRightDT.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        if (robot.tRightDT != null)
            robot.tRightDT.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        waitForStart();

        ObjectDetector.POSITIONS position = detector.getDecision(this);
        //robot.strafeDrive(-40, 0, 0.7, this);
        //robot.strafeDrive(0,70, 0.7, this);
        // robot.strafeDrive(-39, 0, 0.7, this);
        Bot.strafeToPosition(-40,.7);
        Bot.gyroDrive(.7, 70, 70, 70, 70, 0, this);
        Bot.strafeToPosition(39,.7);


        // getting into position to drop cone
        robot.Lift.setTargetPosition(var.Lvl_Tall);
        while (!checktarget(robot.Lift.getCurrentPosition(), robot.Lift.getTargetPosition()))
            sleep(300);
        //robot.strafeDrive(0, 4, 0.7, this);
        Bot.gyroDrive(.7, 4, 4, 4, 4, 0, this);

        robot.Claw.setTargetPosition(var.claw_open);
        sleep(500);
        //robot.strafeDrive(0, -4, 0.7, this);
        Bot.gyroDrive(.7, -4, -4, -4, -4, 0, this);
        robot.Lift.setTargetPosition(var.Lvl_Ground);
        robot.Claw.setTargetPosition(var.claw_zero);
        sleep(1000);
        //robot.strafeDrive(30, 0, 0.7, this);
        //robot.strafeDrive(0, 65, 0.7, this);
        Bot.strafeToPosition(-30,.7);
        Bot.gyroDrive(.7, 65, 65, 65, 65, 0, this);


        // make the decision
        switch (position) {
            case POS1:
                break;
            case POS2:
                //robot.strafeDrive(55, 0, 0.7, this);
                Bot.strafeToPosition(-55,.7);

                break;
            case POS3:
                //robot.strafeDrive(112, 0, 0.7, this);
                Bot.strafeToPosition(-112,.7);

        }
    }

    private boolean checktarget(int currentPos,int desiredPos) {
        int absCurrentPos = Math.abs(currentPos);
        int absDesiredPos = Math.abs(desiredPos);
        int deadband = 4;
        return (absCurrentPos >= (absDesiredPos - deadband) && absCurrentPos <= (absDesiredPos + deadband));
    }
}
