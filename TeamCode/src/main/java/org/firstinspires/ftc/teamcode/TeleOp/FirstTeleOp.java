package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Robot.Bot;
import org.firstinspires.ftc.teamcode.Robot.Variables;


@TeleOp(name="FirstTeleOp")

public class FirstTeleOp extends OpMode {

    Bot robot = null;
    Variables var = null;


    @Override
    public void init() {
        robot = new Bot();
        var = new Variables();
        robot.init(hardwareMap,this);
    }

    public void loop(){
        Drive();
        Controls();
    }

    void Drive(){
        double vertical   =  gamepad1.left_stick_y;
        double horizontal =  gamepad1.left_stick_x;
        double turn = gamepad1.right_stick_x;

        float spd = 3;
        if (gamepad1.a) { var.slo = !var.slo; }
        if (var.slo) { spd = 9; } else { spd = 3; }

        double tLeftPower  = (2 * (-vertical) + horizontal) / spd - turn;
        double bLeftPower  = (2 * ( vertical) + horizontal) / spd + turn;
        double tRightPower = (2 * ( vertical) + horizontal) / spd + turn;
        double bRightPower = (2 * (-vertical) + horizontal) / spd - turn;
        Bot.tLeftDT.setPower( tLeftPower );
        Bot.bLeftDT.setPower( bLeftPower );
        Bot.tRightDT.setPower(tRightPower);
        Bot.bRightDT.setPower(bRightPower);


        telemetry.addLine("top left encoder counts: " + Bot.tLeftDT.getCurrentPosition());
        telemetry.addLine("bottom left encoder counts: " + Bot.bLeftDT.getCurrentPosition());
        telemetry.addLine("top right encoder counts: " + Bot.tRightDT.getCurrentPosition());
        telemetry.addLine("bottom right encoder counts: " + Bot.bRightDT.getCurrentPosition());
    }

    void Controls(){
        float height = gamepad2.left_stick_y;
        boolean dpad_left  = gamepad2.dpad_left,
                dpad_up    = gamepad2.dpad_up,
                dpad_right = gamepad2.dpad_right,
                dpad_down  = gamepad2.dpad_down;

        if (dpad_down) {
            Bot.Lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            Bot.Lift.setTargetPosition(var.Lvl_Ground);
        } else if (dpad_left) {
            Bot.Lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            Bot.Lift.setTargetPosition(var.Lvl_Short);
        } else if (dpad_right) {
            Bot.Lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            Bot.Lift.setTargetPosition(var.Lvl_Mid);
        } else if (dpad_up) {
            Bot.Lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            Bot.Lift.setTargetPosition(var.Lvl_Tall);
        }
        else {
            Bot.Lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            Bot.Lift.setPower(height);
        }
    }
    

}


