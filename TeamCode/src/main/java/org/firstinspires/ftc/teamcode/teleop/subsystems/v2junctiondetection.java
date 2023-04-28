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
import org.opencv.core.Scalar; // also works


public class v2junctiondetection extends OpenCvPipeline{

    Mat HSV = new Mat();
    Mat thresh = new Mat();
//
//    NEED TO TUNE minwidth, yellowLowHSV, yellowHighHSV

    public static int minwidth = 10; //minimum width of closest junction, need to tune
    public static int width = 0;
    public static int camwidth = 1280;
    public static int camheight = 720;
    public enum JunctionVal{
        ONLEFT,
        ONRIGHT,
        ATJUNCTION,
        NOTDETECTED
    }
    public static JunctionVal junctionVal = JunctionVal.NOTDETECTED;

    MatOfPoint biggest;
    double midpoint_rect = 0;

    Scalar rectdisplaycolor = new Scalar(255, 0, 0);//green rectangles on each side

    public static double lowH = 21, lowS = 112, lowV = 100, highH = 64, highS = 203, highV = 255;

    public static Scalar yellowLowHSV= new Scalar(lowH,lowS,lowV);//grip says to use (21,160,50) low and (33, 255, 255) high
    public static Scalar yellowHighHSV = new Scalar(highH,highS,highV); // high and low yellow HSV values


    Telemetry telemetry;

    public v2junctiondetection(Telemetry tele){
        telemetry = tele;
    }

    @Override
    public Mat processFrame(Mat input) {

        Scalar yellowLowHSV= new Scalar(lowH,lowS,lowV);//grip says to use (21,160,50) low and (33, 255, 255) high
        Scalar yellowHighHSV = new Scalar(highH,highS,highV); // high and low yellow HSV values



        Imgproc.cvtColor(input, HSV, Imgproc.COLOR_RGB2HSV); //converting RGB colors to HSV

        Rect rightrect = new Rect(803, 1, 477, 719);
        Rect leftrect = new Rect(1, 1, 477, 719); // rectangle sizes

        Imgproc.rectangle(input, leftrect, rectdisplaycolor, 5);
        Imgproc.rectangle(input, rightrect, rectdisplaycolor, 5);//displays rectangles

        Core.inRange(HSV, yellowLowHSV, yellowHighHSV, HSV);

        List<MatOfPoint> contours = new ArrayList<>();
        Imgproc.findContours(HSV, contours, new Mat(), 0,1);




        if (!contours.isEmpty()) {
            contours.sort(Collections.reverseOrder(Comparator.comparingDouble(m -> Imgproc.boundingRect(m).width))); //orders contours in array from big to small(by width)
            // ____biggest are variables for the biggest on each side
            biggest = contours.get(0); //contour with the largest width(first in the array)
            // use biggest.width to get the width

            Rect rect = Imgproc.boundingRect(biggest);
            if (rect.width > minwidth) {
                Imgproc.rectangle(input, rect.tl(), rect.br(), new Scalar(255,
                        255, 0), 6);
                midpoint_rect = rect.width / 2.0;


                if (midpoint_rect < (camwidth / 2.0)) {
                    junctionVal = JunctionVal.ONLEFT;
                } else if (midpoint_rect > (camwidth / 2.0)) {
                    junctionVal = JunctionVal.ONRIGHT;
                } else {
                    junctionVal = JunctionVal.ATJUNCTION;
                }
            }
        }

        telemetry.addLine("countours :"+ contours.size());
        telemetry.addLine("Midpoint of Bouding Box :"+ midpoint_rect);
        telemetry.addData("Junction status: ",junctionVal);


        HSV.release();
        thresh.release();
        return input;


    }
}