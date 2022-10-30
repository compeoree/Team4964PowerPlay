package org.firstinspires.ftc.teamcode.Auto.Detection;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Robot.Bot;


@Autonomous(name="VISION Test")
public class VisionTest extends LinearOpMode{

    Bot robot = new Bot();

    @Override
    public void runOpMode() throws InterruptedException {

        ObjectDetector detector = new ObjectDetector(this, true,false);

        robot.init(hardwareMap, this);

        //Once Gamepad 1 'A' is pressed, code exits while loop and provides safe stoppage of VISION Test.
        while (!gamepad1.a) {
            ObjectDetector.POSITIONS positionBefore = detector.getDecision();
            telemetry.addData("", positionBefore);
        }

        waitForStart();
    }
}