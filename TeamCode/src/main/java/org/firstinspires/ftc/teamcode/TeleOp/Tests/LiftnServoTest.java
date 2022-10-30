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

    void Lift(){
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
            telemetry.addLine("lift position " + Bot.Lift.getCurrentPosition());
        }

    }

    void Servo(){
        float open = gamepad2.right_trigger,
              close = gamepad2.left_trigger;
        Bot.Claw.setPosition(Bot.Claw.getPosition() + open - close);
        telemetry.addLine("claw position " + Bot.Claw.getPosition());
    }

}
