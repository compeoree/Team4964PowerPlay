package org.firstinspires.ftc.teamcode.Auto.Detection;



import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
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

    private final Point DETECT_BOX_TL   = new Point(140,100); // 320x240, div 2, minus 20
    private final Point DETECT_BOX_BR   = new Point(180, 140);
    private final Scalar DETECT_BOX_COLOR = new Scalar(255, 0, 0);
    private final int BOUNDING_BOX_THICKNESS = 3;

    private COLOR cone;
    private boolean show_value = true;

    public ObjectDetector(OpMode op, boolean isFrontCAM, boolean isRed) {
        opMode = op;

        int cameraMonitorViewId = opMode.hardwareMap.appContext.getResources().getIdentifier(
                "cameraMonitorViewId", "id", opMode.hardwareMap.appContext.getPackageName());

        camera = OpenCvCameraFactory.getInstance().createWebcam(opMode.hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        pipeline = new CustomPipeline();
        camera.openCameraDevice();
        camera.setPipeline(pipeline);
        camera.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT); //Sets resolution and position of webcam
    }

    public void stopStreaming() {
        camera.stopStreaming();
    }

    public enum POSITIONS {
        POS1, POS2, POS3
    }

    public POSITIONS getDecision(OpMode opMode) {
        POSITIONS position = POSITIONS.POS3;

        int r = cone.getRed();
        int g = cone.getGreen();
        int b = cone.getBlue();

        if (r == 255)
            position = POSITIONS.POS3;
        if (b == 255)
            position = POSITIONS.POS2;
        if (g == 255)
            position = POSITIONS.POS1;
        /*if (r > 254 && (g < 70 && b < 70))
            position = POSITIONS.POS3;
        if (g > 254 && (r < 240 && b < 240))
            position = POSITIONS.POS2;
        if (b > 254 && (r < 240 && b < 140))
            position = POSITIONS.POS2;
        int leftValue   = -cone.getBlack();
        int middleValue = cone.getdBlue();
        int rightValue  = cone.getRed();

        if(leftValue > middleValue && leftValue > rightValue){
            position = POSITIONS.POS1;
            opMode.telemetry.addData("decision:", position);
        }
        else if(middleValue > leftValue && middleValue > rightValue){
            position = POSITIONS.POS2;
            opMode.telemetry.addData("decision:", position);
        }
        else{
            opMode.telemetry.addData("decision:", position);
        }*/


        if (show_value){
            opMode.telemetry.addLine("Position: " + position);
            //opMode.telemetry.addData("Position: ", position);
            //opMode.telemetry.addData("Left Value: ", leftValue);
            //opMode.telemetry.addData("Middle Value: ", middleValue);
            //opMode.telemetry.addData("Right Value: ", rightValue);
            opMode.telemetry.update();
        }

        return position;
    }

    class CustomPipeline extends OpenCvPipeline {

        @Override
        public Mat processFrame(Mat input){
            cone = getAverageColor(input, DETECT_BOX_TL, DETECT_BOX_BR);
            Imgproc.rectangle(input, DETECT_BOX_TL, DETECT_BOX_BR, DETECT_BOX_COLOR, BOUNDING_BOX_THICKNESS);
            getDecision(opMode);
            //sendTelemetry();
            return input;
        }

        private COLOR getAverageColor(Mat mat, Point topLeft, Point bottomRight) {
            double red = 0;
            double green = 0;
            double blue = 0;
            double max = 0;

            for (int x = (int)topLeft.x; x < bottomRight.x; x++) {
                for (int y = (int)topLeft.y; y < bottomRight.y; y++) {
                    double[] pixel = mat.get(y,x);
                    red += pixel[0];
                    green += pixel[1];
                    blue += pixel[2];
                }
            }

            //double total = red + green + blue;
            if (max < red)
                max = red;
            if (max < green)
                max = green;
            if (max < blue)
                max = blue;
            red = red / max * 255;
            green = green / max * 255;
            blue = blue / max * 255;

            return new COLOR((int)red, (int)green, (int)blue);
        }

        public void sendTelemetry() {
            opMode.telemetry.addLine(" Blue " + cone.getBlue() + " Green " + cone.getGreen() + " Red " + cone.getRed() + " White " + cone.getWhite() + " Black " + cone.getBlack());
            opMode.telemetry.update();
        }

    }

    public void setTelemShow(boolean show) {
        this.show_value = show;
    }
}

