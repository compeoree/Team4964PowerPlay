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
        Servo();
    }

    void Drive(){
        double vertical   =   gamepad1.left_stick_y;
        double horizontal =  -gamepad1.left_stick_x;
        double turn = -gamepad1.right_stick_x;

        float spd = 3;
        if (gamepad1.a) { var.slo = !var.slo; }
        if (var.slo) { spd = 9; } else { spd = 3; }

        double tLeftPower  = (vertical + horizontal + turn) / spd ;
        double bLeftPower  = (vertical - horizontal + turn) / spd ;
        double tRightPower = (vertical + horizontal - turn) / spd ;
        double bRightPower = (vertical - horizontal - turn) / spd ;
        Bot.tLeftDT.setPower( tLeftPower );
        Bot.bLeftDT.setPower( bLeftPower );
        Bot.tRightDT.setPower(tRightPower);
        Bot.bRightDT.setPower(bRightPower);


        telemetry.addLine("top left encoder counts: " + Bot.tLeftDT.getCurrentPosition());
        telemetry.addLine("bottom left encoder counts: " + Bot.bLeftDT.getCurrentPosition());
        telemetry.addLine("top right encoder counts: " + Bot.tRightDT.getCurrentPosition());
        telemetry.addLine("bottom right encoder counts: " + Bot.bRightDT.getCurrentPosition());
    }

    public boolean assist;

    void Controls(){
        float height = gamepad2.left_stick_y;
        int i = 1;
        boolean dpad_left  = gamepad2.dpad_left,
                dpad_up    = gamepad2.dpad_up,
                dpad_right = gamepad2.dpad_right,
                dpad_down  = gamepad2.dpad_down;

        if(gamepad2.a){assist = !assist;}

        if (!assist) {
            Bot.Lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            Bot.Lift.setPower(height);
            telemetry.addLine("lift position " + Bot.Lift.getCurrentPosition());
            if (gamepad2.b) { telemetry.addLine("#" + i + " saved Lift position " + Bot.Lift.getCurrentPosition()); i += 1; }
        }

        else if (assist) {
            Bot.Lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            if (dpad_down){
                Bot.Lift.setTargetPosition(var.Lvl_Ground);
            }
            else if (dpad_left){
                Bot.Lift.setTargetPosition(var.Lvl_Short);
            }
            else if (dpad_right){
                Bot.Lift.setTargetPosition(var.Lvl_Mid);
            }
            else if (dpad_up) {
                Bot.Lift.setTargetPosition(var.Lvl_Tall);
            }

        }
    }

    void Servo(){
        float close = gamepad2.left_trigger;
        Bot.Claw.setTargetPosition((int)close*100);
        telemetry.addLine("claw position " + Bot.Claw.getCurrentPosition());
    }
    

}


