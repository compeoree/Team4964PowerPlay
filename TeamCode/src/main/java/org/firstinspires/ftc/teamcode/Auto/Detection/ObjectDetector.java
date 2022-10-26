package org.firstinspires.ftc.teamcode.Auto.Detection;



import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;


public class ObjectDetector {

    OpMode opMode;
    OpenCvCamera camera;

    CustomPipeline pipeline;



    private final Point R_FRONT_LEFT_TL   = new Point(110,180);
    private final Point R_FRONT_LEFT_BR   = new Point(160, 220);
    private final Point R_FRONT_MIDDLE_TL = new Point(260, 180);
    private final Point R_FRONT_MIDDLE_BR = new Point(310,  220);
    private final Point R_FRONT_RIGHT_TL  = new Point(10, 180);
    private final Point R_FRONT_RIGHT_BR  = new Point(51, 220);


    private Point leftTL;
    private Point leftBR;
    private Point middleTL;
    private Point middleBR;
    private Point rightTL;
    private Point rightBR;


    private COLOR cone;
    private boolean show_value = true;


    public ObjectDetector(OpMode op, boolean isFrontCAM, boolean isRed){

        opMode = op;

        int cameraMonitorViewId = opMode.hardwareMap.appContext.getResources().getIdentifier(
                "cameraMonitorViewId", "id", opMode.hardwareMap.appContext.getPackageName());

        camera = OpenCvCameraFactory.getInstance().createWebcam(opMode.hardwareMap.get(WebcamName.class, "sideCAM"), cameraMonitorViewId);

        pipeline = new CustomPipeline();
        camera.openCameraDevice();
        camera.setPipeline(pipeline);
        camera.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT); //Sets resolution and position of webcam


            leftTL   = R_FRONT_LEFT_TL;
            leftBR   = R_FRONT_LEFT_BR;
            middleTL = R_FRONT_MIDDLE_TL;
            middleBR = R_FRONT_MIDDLE_BR;
            rightTL  = R_FRONT_RIGHT_TL;
            rightBR  = R_FRONT_RIGHT_BR;

    }

    public void stopStreaming() {
        camera.stopStreaming();
    }

    public enum POSITIONS {
        POS1, POS2, POS3
    }

    public POSITIONS getDecision() {

        POSITIONS position = POSITIONS.POS3;

        int leftValue   = cone.getWhite();
        int middleValue = cone.getBlue();
        int rightValue  = cone.getRed(); //Ensure getBlack works instead of getdBlue

        if(leftValue > middleValue && leftValue > rightValue){
            position = POSITIONS.POS1;
        }
        else if(middleValue > leftValue && middleValue > rightValue){
            position = POSITIONS.POS2;
        }

        if (show_value){

            opMode.telemetry.addData("Position: ", position);
            opMode.telemetry.addData("Left Value: ", leftValue);
            opMode.telemetry.addData("Middle Value: ", middleValue);
            opMode.telemetry.addData("Right Value: ", rightValue);
            opMode.telemetry.update();
        }

        return position;
    }

    class CustomPipeline extends OpenCvPipeline {

        @Override
        public Mat processFrame(Mat input){


            cone = getAverageColor(input, rightTL, rightBR);

            int thickness = 3;
            Scalar leftColor = new Scalar(255, 0, 0);
            Scalar middleColor = new Scalar(255, 255, 255);
            Scalar rightColor = new Scalar(0, 0, 255);

            Imgproc.rectangle(input, leftTL, leftBR, leftColor, thickness);
            Imgproc.rectangle(input, middleTL, middleBR, middleColor, thickness);
            Imgproc.rectangle(input, rightTL, rightBR, rightColor, thickness);

            //sendTelemetry();

            return input;
        }

        private COLOR getAverageColor(Mat mat, Point topLeft, Point bottomRight) {
            int red = 0;
            int green = 0;
            int blue = 0;
            int total = 0;

            for (int x = (int)topLeft.x; x < bottomRight.x; x++){
                for (int y = (int)topLeft.y; y < bottomRight.y; y++){
                    red += mat.get(y,x)[0];
                    green += mat.get(y,x)[1];
                    blue += mat.get(y,x)[2];
                    total++;
                }
            }

            red /= total;
            green /= total;
            blue /= total;

            return new COLOR(red, green, blue);
        }

        public void sendTelemetry() {
            opMode.telemetry.addLine("determining... :" + " Blue " + cone.getBlue() + " White " + cone.getWhite() + " Red " + cone.getRed());

            opMode.telemetry.update();
        }

    }

    public void setTelemShow(boolean show) {
        this.show_value = show;
    }
}

