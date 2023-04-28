package org.firstinspires.ftc.teamcode.teleop.subsystems;

import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;
import java.util.Iterator;
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

    Mat left = new Mat();
    Mat right = new Mat();
    Mat leftthres = new Mat();
    Mat rightthres = new Mat();
    Mat HSV = new Mat();
    Mat outPut = new Mat();
//
//    NEED TO TUNE minwidth, yellowLowHSV, yellowHighHSV

    public static int minwidth = 10; //minimum width of closest junction, need to tune
    public static int leftwidth = 0, rightwidth = 0;
    public enum JunctionVal{
        ONLEFT,
        ONRIGHT,
        ATJUNCTION,
        NOTDETECTED
    }
    public static JunctionVal junctionVal = JunctionVal.NOTDETECTED;

    private final List<MatOfPoint> leftcontours = new ArrayList<>();
    private final List<MatOfPoint> rightcontours = new ArrayList<>(); //arrays for the contours on each side
    MatOfPoint biggest;

    Scalar rectdisplaycolor = new Scalar(255, 0, 0);//green rectangles on each side

    public static double lowH = 21, lowS = 160, lowV = 50, highH = 33, highS = 255, highV = 255;

    public static Scalar yellowLowHSV= new Scalar(lowH,lowS,lowV);//grip says to use (21,160,50) low and (33, 255, 255) high
    public static Scalar yellowHighHSV = new Scalar(highH,highS,highV); // high and low yellow HSV values


    Telemetry telemetry;

    public JunctionDetectionPipeline(Telemetry tele){
        telemetry = tele;
    }

    @Override
    public Mat processFrame(Mat input) {

        Scalar yellowLowHSV= new Scalar(lowH,lowS,lowV);//grip says to use (21,160,50) low and (33, 255, 255) high
        Scalar yellowHighHSV = new Scalar(highH,highS,highV); // high and low yellow HSV values

//        //DELETE ALL THIS WHEN DONE TESTING
//
//        //DELETE HERE
//        int IMG_HEIGHT = input.rows();
//        int IMG_WIDTH = input.cols();
//
//        Imgproc.cvtColor(input, input, Imgproc.COLOR_RGB2HSV);
//
//        Core.inRange(input, yellowLowHSV, yellowHighHSV, input);
//
//        return input;
////TO HERE



        input.copyTo(outPut);

        Imgproc.cvtColor(input, HSV, Imgproc.COLOR_RGB2HSV); //converting RGB colors to HSV

        Rect rightrect = new Rect(803, 1, 477, 719);
        Rect leftrect = new Rect(1, 1, 477, 719); // rectangle sizes

        Imgproc.rectangle(outPut, leftrect, rectdisplaycolor, 5);
        Imgproc.rectangle(outPut, rightrect, rectdisplaycolor, 5);//displays rectangles

        left = HSV.submat(leftrect);
        right = HSV.submat(rightrect);//makes the submats the size of the above rectangles

        Core.inRange(left, yellowLowHSV, yellowHighHSV, outPut);
        Core.inRange(right, yellowLowHSV, yellowHighHSV, outPut);
//
//        leftcontours.clear();
//        Imgproc.findContours(leftthres, leftcontours, new Mat(), Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);
//        //for (MatOfPoint contour : leftcontours) {
//        //    Rect rect = Imgproc.boundingRect(contour);
//        //    if (rect.height > 10 && rect.height < 720) {
//        //        Imgproc.rectangle(outPut, rect.tl(), rect.br(), new Scalar(255,
//        //                0, 0), 1);
//        //    }
//        //}
//
//        rightcontours.clear();
//        Imgproc.findContours(rightthres, rightcontours, new Mat(), Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);
//
////        Imgproc.drawContours(outPut, leftcontours, -1, new Scalar(255, 255, 255));
////        Imgproc.drawContours(outPut, rightcontours, -1, new Scalar(255, 255, 255)); //outlines contours in white
//
//        //telemetry.addLine("Left countours :"+ leftcontours);
//        //telemetry.addLine("Right countours :"+rightcontours);
//
//        if (!leftcontours.isEmpty() && !rightcontours.isEmpty()) {
//            leftcontours.sort(Collections.reverseOrder(Comparator.comparingDouble(m -> Imgproc.boundingRect(m).width))); //orders contours in array from big to small(by width)
//            // ____biggest are variables for the biggest on each side
//            MatOfPoint leftbiggest = leftcontours.get(0); //contour with the largest width(first in the array)
//            // use leftbiggest.width to get the width
//
//            if (leftbiggest.width() > minwidth) {
//                leftwidth = leftbiggest.width();
//            } else {
//                leftwidth = 0;
//            }
//
//            rightcontours.sort(Collections.reverseOrder(Comparator.comparingDouble(m -> Imgproc.boundingRect(m).width)));
//            MatOfPoint rightbiggest = rightcontours.get(0); //contour with the largest width
//            // use rightbiggest.width to get the width
//
//            if (rightbiggest.width() > minwidth) { //if the width on either side passes the threshold, it will put the width into a variable to compare
//                rightwidth = rightbiggest.width();
//            } else {
//                rightwidth = 0;
//            }
//
//            if (leftwidth > rightwidth) {
//                junctionVal = JunctionVal.ONLEFT;
//                biggest = leftbiggest;
//                Rect rect = Imgproc.boundingRect(biggest);
//                if (rect.height > 12 && rect.height < 720) {
//                    Imgproc.rectangle(outPut, rect.tl(), rect.br(), new Scalar(255, 255, 255), 6);
//                }
//                telemetry.addData("ON LEFT", leftwidth);
//            } else if (rightwidth > leftwidth) {
//                junctionVal = JunctionVal.ONRIGHT;
//                biggest = rightbiggest;
//                Rect rect = Imgproc.boundingRect(biggest);
//                if (rect.height > 12 && rect.height < 720) {
//                    Imgproc.rectangle(outPut, rect.tl(), rect.br(), new Scalar(255, 255, 255), 6);
//                }
//                telemetry.addData("ON RIGHT", rightwidth);
//            } else {
//                if (rightwidth == 0) {
//                    junctionVal = JunctionVal.NOTDETECTED;
//                    telemetry.addLine("NOT DETECTED");
//                } else {
//                    junctionVal = JunctionVal.ATJUNCTION;
//                    telemetry.addLine("LINED UP");
//                }
//
//
//            }
//
//        }
//
//        telemetry.update();
//
//
        input.release();
        HSV.release();
        left.release();
        right.release();
        leftthres.release();
        rightthres.release();

        return outPut;


    }
}