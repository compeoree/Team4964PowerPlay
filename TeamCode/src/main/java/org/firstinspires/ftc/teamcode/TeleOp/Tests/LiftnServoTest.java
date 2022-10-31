package org.firstinspires.ftc.teamcode.TeleOp.Tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Robot.Bot;
import org.firstinspires.ftc.teamcode.Robot.Variables;


@TeleOp(name="Lift and Servo test")

public class LiftnServoTest extends OpMode {

    Bot robot = null;
    Variables var = null;
    boolean set_mode = false;

    @Override
    public  void  init(){
        robot = new Bot();
        var = new Variables();
        robot.init(hardwareMap, this);

    }

    public void loop(){
        Lift();
        Servo();
    }

    public boolean assist;
    void Lift(){
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
        int i = 1;
        Bot.Claw.setPosition(close);
        if (gamepad2.x) { telemetry.addLine("#" + i + " saved claw position " + Bot.Claw.getPosition()); i += 1; }
        telemetry.addLine("claw position " + Bot.Claw.getPosition());
    }

}
