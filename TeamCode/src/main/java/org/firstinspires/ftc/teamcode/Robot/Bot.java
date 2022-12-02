package org.firstinspires.ftc.teamcode.Robot;

import static java.lang.Thread.sleep;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;


public class Bot {
    public static DcMotor tLeftDT  = null;
    public static DcMotor bLeftDT  = null;
    public static DcMotor tRightDT = null;
    public static DcMotor bRightDT = null;
    public static DcMotor Lift     = null;
    public static DcMotor Claw     = null;

    public static ModernRoboticsI2cGyro Gyro;

    static final double HEADING_THRESHOLD = 1;      // As tight as we can make it with an integer gyro
    static final double P_TURN_COEFF = 0.1;     // Larger is more responsive, but also less stable
    static final double P_DRIVE_COEFF = 0.07;     // Larger is more responsive, but also less stable

    public static double DRIVE_SPEED = 0.4;
    public static double TURN_SPEED = 0.4;
    public static double amountError = 2;



    public static final double conversion = Variables.conversion; // var.conversion of encoder rotations to centimetres
    public static final double tileConversion = conversion * 60; // var.conversion of encoder rotations to tiles !!EDIT!!

    public void init(HardwareMap ahwMap, OpMode opMode) {
        HardwareMap hwMap = ahwMap;
        Variables var = new Variables();
        tLeftDT   = hwMap.get(DcMotor.class, "FrontL");
        bLeftDT   = hwMap.get(DcMotor.class, "BackL");
        tRightDT  = hwMap.get(DcMotor.class, "FrontR");
        bRightDT  = hwMap.get(DcMotor.class, "BackR");
        Lift      = hwMap.get(DcMotor.class, "lift"    );
        Claw      = hwMap.get(DcMotor.class, "claw"    );


        bLeftDT.setDirection(DcMotor.Direction.FORWARD);
        tLeftDT.setDirection(DcMotor.Direction.FORWARD);
        bRightDT.setDirection(DcMotor.Direction.REVERSE);
        bLeftDT.setDirection(DcMotor.Direction.FORWARD);
        Lift.setDirection(DcMotor.Direction.FORWARD);
        Claw.setDirection(DcMotor.Direction.FORWARD);



        bLeftDT.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        tLeftDT.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bRightDT.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bLeftDT.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Claw.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);



        bLeftDT.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        tLeftDT.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bRightDT.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bLeftDT.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Claw.setMode(DcMotor.RunMode.RUN_USING_ENCODER);



        bLeftDT.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        tLeftDT.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bRightDT.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        tRightDT.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Claw.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);




        bLeftDT.setPower(0);
        tLeftDT.setPower(0);
        bRightDT.setPower(0);
        bLeftDT.setPower(0);
        Lift.setPower(0);
        Claw.setPower(0);

        Claw.setTargetPosition(var.claw_cone);









        opMode.telemetry.addLine("Initialization Complete! ;) ");
        opMode.telemetry.update();



    }



    //driving using only Mecanum strafe
    public static void strafeDrive (float distance, double speed, LinearOpMode opMode) throws InterruptedException
    {
        // if it breaks do this https://github.com/AnishJag/FTCFreightFrenzy/blob/master/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/Base/MainBase.java
        if (opMode.opModeIsActive()) {

            distance = (float) (1 * Math.pow(distance, 3));

            int tLeftPos =   tLeftDT.getCurrentPosition() + (int) (conversion * -distance);
            int bLeftPower =   bLeftDT.getCurrentPosition() + (int) (conversion *  distance);
            int tRightPower = tRightDT.getCurrentPosition() + (int) (conversion *  distance);
            int bRightPower = bRightDT.getCurrentPosition() + (int) (conversion * -distance);

            tLeftDT.setTargetPosition(tLeftPos);
            bLeftDT.setTargetPosition(bLeftPower);
            tRightDT.setTargetPosition(tRightPower);
            bRightDT.setTargetPosition(bRightPower);

            tLeftDT.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            bLeftDT.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            tRightDT.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            bRightDT.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            opMode.telemetry.addLine("Front left encoder counts: " + tLeftDT.getCurrentPosition() + ":" + tLeftDT.getTargetPosition());
            opMode.telemetry.addLine("Back left encoder counts: " + bLeftDT.getCurrentPosition() + ":" + bLeftDT.getTargetPosition());
            opMode.telemetry.addLine("Front right encoder counts: " + tRightDT.getCurrentPosition() + ":" + tRightDT.getTargetPosition());
            opMode.telemetry.addLine("Back right encoder counts: " + bRightDT.getCurrentPosition() + ":" + bRightDT.getTargetPosition());
            opMode.telemetry.update();

            double rangedSpeed = Range.clip(Math.abs(speed), 0.0, 1.0);
            tLeftDT.setPower(Math.abs(rangedSpeed));
            tRightDT.setPower(Math.abs(rangedSpeed));
            bLeftDT.setPower(Math.abs(rangedSpeed));
            bRightDT.setPower(Math.abs(rangedSpeed));

            //opMode.telemetry.addLine("Lift encoder counts: " + Lift.getCurrentPosition());
            //opMode.telemetry.addLine("Claw encoder counts: " + Claw.getCurrentPosition());

            // check for in position (could also use isBusy() to check each motor)
            while (!checkTarget(tLeftDT.getCurrentPosition(), tLeftPos)) {
                sleep(100);
                opMode.telemetry.addLine("Front left encoder counts: " + tLeftDT.getCurrentPosition() + ":" + tLeftDT.getTargetPosition());
                opMode.telemetry.addLine("Back left encoder counts: " + bLeftDT.getCurrentPosition() + ":" + bLeftDT.getTargetPosition());
                opMode.telemetry.addLine("Front right encoder counts: " + tRightDT.getCurrentPosition() + ":" + tRightDT.getTargetPosition());
                opMode.telemetry.addLine("Back right encoder counts: " + bRightDT.getCurrentPosition() + ":" + bRightDT.getTargetPosition());
                opMode.telemetry.update();
            }
        }

    }
    public static void straightDrive (float distance, double speed, LinearOpMode opMode) throws InterruptedException
    {
        // if it breaks do this https://github.com/AnishJag/FTCFreightFrenzy/blob/master/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/Base/MainBase.java
        if (opMode.opModeIsActive()) {

            distance = (float) (1 * Math.pow(distance, 3));

            int tLeftPos =   tLeftDT.getCurrentPosition() + (int) (conversion * distance);
            int bLeftPower =   bLeftDT.getCurrentPosition() + (int) (conversion *  distance);
            int tRightPower = tRightDT.getCurrentPosition() + (int) (conversion *  distance);
            int bRightPower = bRightDT.getCurrentPosition() + (int) (conversion * distance);

            tLeftDT.setTargetPosition(tLeftPos);
            bLeftDT.setTargetPosition(bLeftPower);
            tRightDT.setTargetPosition(tRightPower);
            bRightDT.setTargetPosition(bRightPower);

            tLeftDT.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            bLeftDT.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            tRightDT.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            bRightDT.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            opMode.telemetry.addLine("Front left encoder counts: " + tLeftDT.getCurrentPosition() + ":" + tLeftDT.getTargetPosition());
            opMode.telemetry.addLine("Back left encoder counts: " + bLeftDT.getCurrentPosition() + ":" + bLeftDT.getTargetPosition());
            opMode.telemetry.addLine("Front right encoder counts: " + tRightDT.getCurrentPosition() + ":" + tRightDT.getTargetPosition());
            opMode.telemetry.addLine("Back right encoder counts: " + bRightDT.getCurrentPosition() + ":" + bRightDT.getTargetPosition());
            opMode.telemetry.update();

            double rangedSpeed = Range.clip(Math.abs(speed), 0.0, 1.0);
            tLeftDT.setPower(Math.abs(rangedSpeed));
            tRightDT.setPower(Math.abs(rangedSpeed));
            bLeftDT.setPower(Math.abs(rangedSpeed));
            bRightDT.setPower(Math.abs(rangedSpeed));

            //opMode.telemetry.addLine("Lift encoder counts: " + Lift.getCurrentPosition());
            //opMode.telemetry.addLine("Claw encoder counts: " + Claw.getCurrentPosition());

            // check for in position (could also use isBusy() to check each motor)
            while (!checkTarget(tLeftDT.getCurrentPosition(), tLeftPos)) {
                sleep(100);
                opMode.telemetry.addLine("Front left encoder counts: " + tLeftDT.getCurrentPosition() + ":" + tLeftDT.getTargetPosition());
                opMode.telemetry.addLine("Back left encoder counts: " + bLeftDT.getCurrentPosition() + ":" + bLeftDT.getTargetPosition());
                opMode.telemetry.addLine("Front right encoder counts: " + tRightDT.getCurrentPosition() + ":" + tRightDT.getTargetPosition());
                opMode.telemetry.addLine("Back right encoder counts: " + bRightDT.getCurrentPosition() + ":" + bRightDT.getTargetPosition());
                opMode.telemetry.update();
            }
        }

    }

//    public static void driveStraight(double speed, double fLeftcm, double fRightcm, double bLeftcm,
//                                     double bRightcm, LinearOpMode opmode){
//
//        int newFrontLeftTarget  = tLeftDT.getCurrentPosition()  + (int) (fLeftcm * conversion);
//        int newFrontRightTarget = tRightDT.getCurrentPosition() + (int) (fRightcm * conversion);
//        int newBackLeftTarget   = bLeftDT.getCurrentPosition()  + (int) (bLeftcm * conversion);
//        int newBackRightTarget  = bRightDT.getCurrentPosition() + (int) (bRightcm * conversion);
//
//
//        // Set Target and Turn On RUN_TO_POSITION
//        tLeftDT.setTargetPosition(newFrontLeftTarget);
//        tRightDT.setTargetPosition(newFrontRightTarget);
//        bLeftDT.setTargetPosition(newBackLeftTarget);
//        bRightDT.setTargetPosition(newBackRightTarget);
//
//        tLeftDT.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        tRightDT.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        bLeftDT.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        bRightDT.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//
//        speed = Range.clip(Math.abs(speed), 0.0, 1.0);
//        tLeftDT.setPower(Math.abs(speed));
//        tRightDT.setPower(Math.abs(speed));
//        bLeftDT.setPower(Math.abs(speed));
//        bRightDT.setPower(Math.abs(speed));
//    }

    public static void gyroDrive(double speed,
                                   double fLeftcm, double fRightcm, double bLeftcm,
                                   double bRightcm,
                                   double angle, LinearOpMode opMode) {

            int newFrontLeftTarget;
            int newFrontRightTarget;
            int newBackLeftTarget;
            int newBackRightTarget;

            double HalfMaxOne;
            double HalfMaxTwo;

            double max;

            double error;
            double steer;
            double frontLeftSpeed;
            double frontRightSpeed;
            double backLeftSpeed;
            double backRightSpeed;

            double ErrorAmount;
            boolean goodEnough = false;



            // Determine new target position, and pass to motor controller
            newFrontLeftTarget = tLeftDT.getCurrentPosition() + (int) (fLeftcm * conversion);
            newFrontRightTarget = tRightDT.getCurrentPosition() + (int) (fRightcm * conversion);
            newBackLeftTarget = bLeftDT.getCurrentPosition() + (int) (bLeftcm * conversion);
            newBackRightTarget = bRightDT.getCurrentPosition() + (int) (bRightcm * conversion);


            // Set Target and Turn On RUN_TO_POSITION
            tLeftDT.setTargetPosition(newFrontLeftTarget);
            tRightDT.setTargetPosition(newFrontRightTarget);
            bLeftDT.setTargetPosition(newBackLeftTarget);
            bRightDT.setTargetPosition(newBackRightTarget);

            tLeftDT.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            tRightDT.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            bLeftDT.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            bRightDT.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // start motion.
            speed = Range.clip(Math.abs(speed), 0.0, 1.0);
            tLeftDT.setPower(Math.abs(speed));
            tRightDT.setPower(Math.abs(speed));
            bLeftDT.setPower(Math.abs(speed));
            bRightDT.setPower(Math.abs(speed));
            // keep looping while we are still active, and BOTH motors are running.
            while (((tLeftDT.isBusy() && tRightDT.isBusy()) &&
                    (bLeftDT.isBusy() && bRightDT.isBusy())) && !goodEnough) {


                // adjust relative speed based on heading error.
                error = getError(angle);
                steer = getSteer(error, P_DRIVE_COEFF);

                // if driving in reverse, the motor correction also needs to be reversed
                if (fLeftcm < 0 && fRightcm < 0 && bLeftcm < 0 && bRightcm < 0)
                    steer *= -1.0;

                frontLeftSpeed = speed - steer;
                backLeftSpeed = speed - steer;
                backRightSpeed = speed + steer;
                frontRightSpeed = speed + steer;

                // Normalize speeds if either one exceeds +/- 1.0;
                HalfMaxOne = Math.max(Math.abs(frontLeftSpeed), Math.abs(backLeftSpeed));
                HalfMaxTwo = Math.max(Math.abs(frontRightSpeed), Math.abs(backRightSpeed));
                max = Math.max(Math.abs(HalfMaxOne), Math.abs(HalfMaxTwo));
                if (max > 1.0) {
                    frontLeftSpeed /= max;
                    frontRightSpeed /= max;
                    backLeftSpeed /= max;
                    backRightSpeed /= max;
                }

                tLeftDT.setPower(frontLeftSpeed);
                tRightDT.setPower(frontRightSpeed);
                bLeftDT.setPower(backLeftSpeed);
                bRightDT.setPower(backRightSpeed);

                // Display drive status for the driver.
                opMode.telemetry.addData("Err/St", "%5.1f/%5.1f", error, steer);
                opMode.telemetry.addData("Target", "%7d:%7d", newBackLeftTarget, newBackRightTarget, newFrontLeftTarget, newFrontRightTarget);
                opMode.telemetry.addData("Actual", "%7d:%7d", bLeftDT.getCurrentPosition(), bRightDT.getCurrentPosition(), tLeftDT.getCurrentPosition(), tRightDT.getCurrentPosition());
                opMode.telemetry.addData("Speed", "%5.2f:%5.2f", backLeftSpeed, backRightSpeed, frontLeftSpeed, frontRightSpeed);
                opMode.telemetry.update();

                ErrorAmount = ((Math.abs(((newBackLeftTarget) - (bLeftDT.getCurrentPosition())))
                        + (Math.abs(((newFrontLeftTarget) - (tLeftDT.getCurrentPosition()))))
                        + (Math.abs((newBackRightTarget) - (bRightDT.getCurrentPosition())))
                        + (Math.abs(((newFrontRightTarget) - (tRightDT.getCurrentPosition()))))) / conversion);
                if (ErrorAmount < amountError) {
                    goodEnough = true;
                }


            // Stop all motion;
            tLeftDT.setPower(0);
            tRightDT.setPower(0);
            bLeftDT.setPower(0);
            bRightDT.setPower(0);

            // Turn off RUN_TO_POSITION
            tLeftDT.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            tRightDT.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            bLeftDT.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            bRightDT.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }




    }

    public static void strafeToPosition(double cm, double speed) {
        //
        int move = (int) (Math.round(cm * conversion * 0.8));

        //
         bLeftDT.setTargetPosition( bLeftDT.getCurrentPosition() - move);
         tLeftDT.setTargetPosition( tLeftDT.getCurrentPosition() + move);
         bRightDT.setTargetPosition( bRightDT.getCurrentPosition() + move);
         tRightDT.setTargetPosition( tRightDT.getCurrentPosition() - move);
        //
         tLeftDT.setMode(DcMotor.RunMode.RUN_TO_POSITION);
         tRightDT.setMode(DcMotor.RunMode.RUN_TO_POSITION);
         bLeftDT.setMode(DcMotor.RunMode.RUN_TO_POSITION);
         bRightDT.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //
         tLeftDT.setPower(speed);
         bLeftDT.setPower(speed);
         tRightDT.setPower(speed);
         bRightDT.setPower(speed);
        //
        while ( tLeftDT.isBusy() &&  tRightDT.isBusy() &&  bLeftDT.isBusy() &&  bRightDT.isBusy()) {

        }
         tRightDT.setPower(0);
         tLeftDT.setPower(0);
         bRightDT.setPower(0);
         bLeftDT.setPower(0);

    }

    public static double getError(double targetAngle) {

        double robotError;

        // calculate error in -179 to +180 range  (
        robotError = targetAngle - Gyro.getHeading();
        while (robotError > 180) robotError -= 360;
        while (robotError <= -180) robotError += 360;
        return -robotError;
    }

    public static double getSteer(double err, double PCoeff) {
        return Range.clip(err * PCoeff, -DRIVE_SPEED, 1);
    }

    private static boolean checkTarget(int currentPos,int desiredPos) {
        int absCurrentPos = Math.abs(currentPos);
        int absDesiredPos = Math.abs(desiredPos);
        int deadband = 4;
        return (absCurrentPos >= (absDesiredPos - deadband) && absCurrentPos <= (absDesiredPos + deadband));
    }

}


