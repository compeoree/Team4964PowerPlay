package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Robot.Bot;

@TeleOp(name="FirstTeleOp")

public class FirstTeleOp extends OpMode {

    Bot robot = null;
    int Lvl_Ground = 0,
        Lvl_Short  = 3850,
        Lvl_Mid    = 6250,
        Lvl_Tall   = 12890;
    boolean AutoControls = false;

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

        double tLeftPower  =  horizontal + 2 * ( vertical *  horizontal );
        double bLeftPower  =  horizontal + 2 * ( vertical *  horizontal );
        double tRightPower =  horizontal + 2 * ( -vertical *  horizontal );
        double bRightPower =  horizontal + 2 * ( -vertical *  horizontal );

        Bot.tLeftDT.setPower( tLeftPower );
        Bot.bLeftDT.setPower( bLeftPower );
        Bot.tRightDT.setPower(tRightPower);
        Bot.bRightDT.setPower(bRightPower);
    }

    void Controls(){
        //toggle assist mode\\
        boolean Typeset = gamepad2.a;
        if(Typeset){
            AutoControls = !AutoControls ;
        }

        boolean dpad_left  = gamepad2.dpad_left,
                dpad_up    = gamepad2.dpad_up,
                dpad_right = gamepad2.dpad_right,
                dpad_down  = gamepad2.dpad_down;
        float LiftHeight = gamepad2.left_stick_y;
        
        //ASSIST MODE\\
        if(AutoControls) {
            Bot.Lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            if (dpad_down) {
                Bot.Lift.setTargetPosition(Lvl_Ground);
            } else if (dpad_left) {
                Bot.Lift.setTargetPosition(Lvl_Short);
            } else if (dpad_right) {
                Bot.Lift.setTargetPosition(Lvl_Mid);
            } else if (dpad_up) {
                Bot.Lift.setTargetPosition(Lvl_Tall);
            }
        }
        //CONTROL MODE\\
        if(!AutoControls) {
            Bot.Lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            Bot.Lift.setPower(LiftHeight);
        }
        
    }
    

}


