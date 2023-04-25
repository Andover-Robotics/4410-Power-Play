package org.firstinspires.ftc.teamcode.teleop.subsystems;

import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;
import java.util.List;

import org.opencv.core.Mat;
import org.opencv.core.Core;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Rect;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core; // works with eocv-sim
import org.opencv.core.Mat; // also works
import org.opencv.core.Scalar; // also works


public class JunctionDetectionPipeline extends OpenCvPipeline{

    Mat left;
    Mat right;
    Mat leftthres = new Mat();
    Mat rightthres = new Mat();

    //NEED TO TUNE minwidth, yellowLowHSV, yellowHighHSV

    public int minwidth = 90; //minimum width of closest junction, need to tune
    public int leftwidth = 0, rightwidth = 0;
    public enum JunctionVal{
        ONLEFT,
        ONRIGHT,
        ATJUNCTION,
        NOTDETECTED
    }
    public JunctionVal junctionVal = JunctionVal.NOTDETECTED;

    private MatOfPoint rightbiggest, leftbiggest; //variables for the biggest on each side

    private final List<MatOfPoint> leftcontours = new ArrayList<>();
    private final List<MatOfPoint> rightcontours = new ArrayList<>(); //arrays for the contours on each side

    Mat outPut = new Mat();
    Scalar rectdisplaycolor = new Scalar(0, 255, 0);//green rectangles on each side

    Scalar yellowLowHSV= new Scalar(21,160,50);
    Scalar yellowHighHSV = new Scalar(33,255,255); // high and low yellow HSV values
    Telemetry telemetry;

    public JunctionDetectionPipeline(Telemetry telemetry){
        this.telemetry = telemetry;
    }
    @Override

    public Mat processFrame(Mat input) {
        Imgproc.cvtColor(input, input, Imgproc.COLOR_RGB2HSV); //converting RGB colors to HSV

        Rect rightrect = new Rect(803, 1, 476, 720);//reduced width by 1 to check if it was too big
        Rect leftrect = new Rect(1,1,477, 720); // rectangle sizes

        input.copyTo(outPut);
        Imgproc.rectangle(outPut, leftrect, rectdisplaycolor, 3);
        Imgproc.rectangle(outPut, rightrect, rectdisplaycolor, 3);//displays rectangles

        left = input.submat(leftrect);
        right = input.submat(rightrect);//makes the submats the size of the above rectangles

        Core.inRange(left, yellowLowHSV, yellowHighHSV, leftthres);
        Core.inRange(right, yellowLowHSV, yellowHighHSV, rightthres);

        leftcontours.clear();
        Imgproc.findContours(leftthres, leftcontours, new Mat(), Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);
        Imgproc.drawContours(input, leftcontours, -1, new Scalar(255, 0, 0)); //outlines in red

        rightcontours.clear();
        Imgproc.findContours(leftthres, rightcontours, new Mat(), Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);
        Imgproc.drawContours(input, rightcontours, -1, new Scalar(255, 0, 0)); //outlines in red

        if (!leftcontours.isEmpty()) {
            leftcontours.sort(Collections.reverseOrder(Comparator.comparingDouble(m -> Imgproc.boundingRect(m).width))); //orders contours in array from big to small(by width)
            leftbiggest = leftcontours.get(0); //contour with the largest width(first in the array)
            // use leftbiggest.width to get the width
        }
        if (leftbiggest.width() > minwidth) {
            leftwidth = leftbiggest.width();
        }
        else{
            leftwidth = 0;
        }

        if (!rightcontours.isEmpty()) {
            rightcontours.sort(Collections.reverseOrder(Comparator.comparingDouble(m -> Imgproc.boundingRect(m).width)));
            rightbiggest = rightcontours.get(0); //contour with the largest width
            // use rightbiggest.width to get the width
        }
        if (rightbiggest.width() > minwidth) { //if the width on either side passes the threshold, it will put the width into a variable to compare
            rightwidth = rightbiggest.width();
        }
        else{
            rightwidth = 0;
        }

        if (leftwidth>rightwidth){
            junctionVal = JunctionVal.ONLEFT;
            telemetry.addData("ON LEFT", leftwidth);
        }
        else if(rightwidth>leftwidth){
            junctionVal = JunctionVal.ONRIGHT;
            telemetry.addData("ON RIGHT", rightwidth);
        }
        else{
            if(rightwidth == 0){
                junctionVal = JunctionVal.NOTDETECTED;
                telemetry.addLine("NOT DETECTED");
            }
            else{
                junctionVal = JunctionVal.ATJUNCTION;
                telemetry.addLine("LINED UP");
            }
        }

        input.release();
        left.release();
        right.release();
        leftthres.release();
        rightthres.release();

        return outPut;


    }
}