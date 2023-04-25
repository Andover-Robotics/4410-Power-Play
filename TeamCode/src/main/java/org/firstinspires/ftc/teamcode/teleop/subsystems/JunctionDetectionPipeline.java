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
    Scalar rectdisplaycolor = new Scalar(255, 0, 0);//green rectangles on each side

    Scalar yellowLowRGB= new Scalar(21,160,50);
    Scalar yellowHighRGB = new Scalar(0,255,255); // high and low yellow RGB values
    Telemetry telemetry;

    public JunctionDetectionPipeline(Telemetry tele){
        telemetry = tele;
    }

    @Override
    public Mat processFrame(Mat input) {
//        Imgproc.cvtColor(input, input, Imgproc.COLOR_RGB2RGB); //converting RGB colors to RGB

        Rect rightrect = new Rect(803, 1, 476, 719);//reduced width by 1 to check if it was too big
        Rect leftrect = new Rect(1, 1, 477, 719); // rectangle sizes

        input.copyTo(outPut);
        Imgproc.rectangle(outPut, leftrect, rectdisplaycolor, 3);
        Imgproc.rectangle(outPut, rightrect, rectdisplaycolor, 3);//displays rectangles

        left = input.submat(leftrect);
        right = input.submat(rightrect);//makes the submats the size of the above rectangles

        Core.inRange(left, yellowLowRGB, yellowHighRGB, leftthres);
        Core.inRange(right, yellowLowRGB, yellowHighRGB, rightthres);

        leftcontours.clear();
        Imgproc.findContours(leftthres, leftcontours, new Mat(), Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);
//        double maxheight = object_max_size * this.in.height() / 100;
//        double minheight = object_min_size * this.in.height() / 100;
        for (MatOfPoint contour : leftcontours) {
            Rect rect = Imgproc.boundingRect(contour);
            if (rect.height > 10 && rect.height < 720) {
                Imgproc.rectangle(outPut, rect.tl(), rect.br(), new Scalar(255,
                        0, 0), 1);
            }
        }

        rightcontours.clear();
        Imgproc.findContours(rightthres, rightcontours, new Mat(), Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);
//        double maxheight = object_max_size * this.in.height() / 100;
//        double minheight = object_min_size * this.in.height() / 100;
        for (MatOfPoint contour : rightcontours) {
            Rect rect = Imgproc.boundingRect(contour);
            if (rect.height > 10 && rect.height < 720) {
                Imgproc.rectangle(outPut, rect.tl(), rect.br(), new Scalar(255,
                        0, 0), 1);
            }
        }
//        Imgproc.drawContours(outPut, leftcontours, -1, new Scalar(255, 0, 0)); //outlines in red
//        Imgproc.boundingRect(leftcontours);

//        rightcontours.clear();
//        Imgproc.findContours(rightthres, rightcontours, new Mat(), Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);
//        Imgproc.drawContours(outPut, rightcontours, -1, new Scalar(255, 255, 255)); //outlines in red

        telemetry.addLine("Left countours :"+ leftcontours);
        telemetry.addLine("Right countours :"+rightcontours);

        if (!leftcontours.isEmpty() && !rightcontours.isEmpty()) {
            leftcontours.sort(Collections.reverseOrder(Comparator.comparingDouble(m -> Imgproc.boundingRect(m).width))); //orders contours in array from big to small(by width)
            leftbiggest = leftcontours.get(0); //contour with the largest width(first in the array)
            // use leftbiggest.width to get the width

            if (leftbiggest.width() > minwidth) {
                leftwidth = leftbiggest.width();
            } else {
                leftwidth = 0;
            }

            rightcontours.sort(Collections.reverseOrder(Comparator.comparingDouble(m -> Imgproc.boundingRect(m).width)));
            rightbiggest = rightcontours.get(0); //contour with the largest width
            // use rightbiggest.width to get the width

            if (rightbiggest.width() > minwidth) { //if the width on either side passes the threshold, it will put the width into a variable to compare
                rightwidth = rightbiggest.width();
            } else {
                rightwidth = 0;
            }

            if (leftwidth > rightwidth) {
                junctionVal = JunctionVal.ONLEFT;
                telemetry.addData("ON LEFT", leftwidth);
            } else if (rightwidth > leftwidth) {
                junctionVal = JunctionVal.ONRIGHT;
                telemetry.addData("ON RIGHT", rightwidth);
            } else {
                if (rightwidth == 0) {
                    junctionVal = JunctionVal.NOTDETECTED;
                    telemetry.addLine("NOT DETECTED");
                } else {
                    junctionVal = JunctionVal.ATJUNCTION;
                    telemetry.addLine("LINED UP");
                }
            }
        }
        telemetry.update();


        input.release();
        left.release();
        right.release();
        leftthres.release();
        rightthres.release();

        return outPut;


    }
}