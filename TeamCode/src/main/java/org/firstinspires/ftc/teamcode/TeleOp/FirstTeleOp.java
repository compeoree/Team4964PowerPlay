package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Robot.Bot;

@TeleOp(name="FirstTeleOp")

public class FirstTeleOp extends OpMode {

    Bot robot = null;



    @Override
    public void init() {
        robot = new Bot();
        robot.init(hardwareMap,this);
    }

    public void loop(){
        Drive();
        Controls(/*fill out*/);
    }

    void Drive(){
        double vertical   = gamepad1.left_stick_y;
        double horizontal = gamepad1.left_stick_x;

        double tLeftPower  =  horizontal + 2 * ( vertical *    horizontal   );
        double bLeftPower  = -horizontal + 2 * ( vertical * ( -horizontal ) );
        double tRightPower = -horizontal + 2 * ( vertical * ( -horizontal ) );
        double bRightPower =  horizontal + 2 * ( vertical *    horizontal   );

        robot.tLeftDT.setPower( tLeftPower );
        robot.bLeftDT.setPower( bLeftPower );
        robot.tRightDT.setPower(tRightPower);
        robot.bRightDT.setPower(bRightPower);
    }

    void Controls(){
        //whatever controls necessary for the robot
    }

}


