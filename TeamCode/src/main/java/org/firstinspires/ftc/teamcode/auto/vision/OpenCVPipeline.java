package org.firstinspires.ftc.teamcode.auto.vision;

//import com.acmerobotics.dashboard.FtcDashboard;
//import com.acmerobotics.dashboard.telemetry.TelemetryPacket;

import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core; // works with eocv-sim
import org.opencv.core.Mat; // also works
import org.opencv.core.Scalar; // also works
import org.opencv.imgproc.Imgproc; //
import org.openftc.easyopencv.OpenCvPipeline;


public class OpenCVPipeline extends OpenCvPipeline{
    Telemetry telemetry;
    public static double greenPercentage;
    public static double yellowPercentage;
    public static double pinkPercentage;
    public enum SignalVal{
        GREEN,
        YELLOW,
        PINK
    }
    private SignalVal signalVal;
    static Point pointOne = new Point(140,360);
    static Point pointTwo = new Point(180,360);
    static final Rect ROI = new Rect(pointOne, pointTwo);

    public OpenCVPipeline(Telemetry telemetry){
        telemetry= this.telemetry;
    }

    @Override
    public Mat processFrame(Mat input) {
        Scalar greenLowHSV= new Scalar(70,105,133);
        Scalar greenHighHSV = new Scalar(128,255,255);
        Scalar yellowLowHSV= new Scalar(23,78,202);
        Scalar yellowHighHSV = new Scalar(76,220,249);
        Scalar pinkLowHSV= new Scalar(60,96,170);
        Scalar pinkHighHSV = new Scalar(180,179,255);


        Core.inRange(input, greenLowHSV, greenHighHSV, input);
        //modifies grayscaleMat by creating a grayscale version of matrix with the specified color isolated

//        Mat epicSubmat = input.submat(ROI);
        //epicmatSubmat holds a small portion of grayscaleMat (portion is defined by ROI points)

        greenPercentage = Math.round((Core.sumElems(input).val[0]/ROI.area()/255)*100);
        //calculates what percent of epicSubmat is the color green

        Core.inRange(input, yellowLowHSV, yellowHighHSV, input);
//        epicSubmat = input.submat(ROI);
        yellowPercentage = Math.round((Core.sumElems(input).val[0]/ROI.area()/255)*100);

        Core.inRange(input,pinkLowHSV,pinkHighHSV,input);
//        epicSubmat = input.submat(ROI);
        pinkPercentage = Math.round((Core.sumElems(input).val[0]/ROI.area()/255)*100);


        input.release();
        boolean isGreen = greenPercentage>=50;
        //identifies if given color is in the image

        boolean isBlue = yellowPercentage>=50;
        boolean isRed = pinkPercentage>=50;


        if(isGreen){
            signalVal = SignalVal.GREEN;
            telemetry.addData("The signal value is green with a percentage of", greenPercentage);
            telemetry.update();
        }else if(isBlue){
            signalVal = SignalVal.YELLOW;
            telemetry.addData("The signal value is blue with a percentage of",yellowPercentage);
            telemetry.update();
        }else if(isRed){
            signalVal = SignalVal.PINK;
            telemetry.addData("The signal value is red with a percentage of",pinkPercentage);
            telemetry.update();
        }

        return input;
    }
    public SignalVal getSignalVal(){
        return signalVal;
    }
}