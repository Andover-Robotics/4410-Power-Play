package org.firstinspires.ftc.teamcode.auto;

import org.opencv.core.Mat;
import org.opencv.core.Core;
import org.opencv.core.Rect;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core; // works with eocv-sim
import org.opencv.core.Mat; // also works
import org.opencv.core.Scalar; // also works


public class JunctionDetectionPipeline extends OpenCvPipeline{
    Telemetry telemetry;
    public static double yellowPercentage;
    public static int horizOffset = -20, vertOffset = 40, width = 50, height = 50;
    public enum JunctionVal{
        CLOSE_TO,
        CLOSER_TO,
        AT_JUNCTION,
        NOT_DETECTED
    }
    private JunctionVal junctionVal = JunctionVal.NOT_DETECTED;
    static Rect ROI = new Rect();

    public JunctionDetectionPipeline(Telemetry telemetry){
        this.telemetry = telemetry;
    }
    Scalar yellowLowHSV= new Scalar(19,112, 73);
    Scalar yellowHighHSV = new Scalar(56,255,255);
    //do not know yellow vals


    Mat smallMat = new Mat();

    @Override
    public Mat processFrame(Mat input) {
        //176 x 144= matrix size
        Imgproc.cvtColor(input, input, Imgproc.COLOR_RGB2HSV);
        ROI.x = input.width()/2 + horizOffset;
        ROI.y = input.height()/2 + vertOffset;
        ROI.width = width + 25;
        ROI.height = height + 25;

        smallMat = input.submat(ROI);
        Mat yellowMat = new Mat(ROI.x, ROI.y, input.type());

        Core.inRange(smallMat, yellowLowHSV, yellowHighHSV, yellowMat);
        //yellow mat has the output array with the isolated yellow color defined between the yellowHSV range

        yellowPercentage = (Core.sumElems(yellowMat)).val[0]/25344;
        yellowMat.release();
        smallMat.release();

        if(yellowPercentage>=30 && yellowPercentage<40){
            junctionVal = JunctionVal.CLOSE_TO;
            telemetry.addData("Junction is approaching", yellowPercentage);
        }else if(yellowPercentage>=40 && yellowPercentage<70){
            junctionVal = JunctionVal.CLOSER_TO;
            telemetry.addData("Junction is closer", yellowPercentage);
        }else if(yellowPercentage>=70){
            junctionVal = JunctionVal.AT_JUNCTION;
            telemetry.addData("At junction", yellowPercentage);
        }else{
            telemetry.addData("Junction has not been detected", yellowPercentage);
        }
        input.release();

        return input;


    }
    public JunctionVal getJunctionVal(){
        if(yellowPercentage>=30 && yellowPercentage<40){
            junctionVal = JunctionVal.CLOSE_TO;
            telemetry.addData("Junction is approaching", yellowPercentage);
        }else if(yellowPercentage>=40 && yellowPercentage<70){
            junctionVal = JunctionVal.CLOSER_TO;
            telemetry.addData("Junction is closer", yellowPercentage);
        }else if(yellowPercentage>=70){
            junctionVal = JunctionVal.AT_JUNCTION;
            telemetry.addData("At junction", yellowPercentage);
        }else{
            telemetry.addData("Junction has not been detected", yellowPercentage);
        }
        return junctionVal;
    }

    public double getYellowPercentage(){
        return yellowPercentage;
    }
}