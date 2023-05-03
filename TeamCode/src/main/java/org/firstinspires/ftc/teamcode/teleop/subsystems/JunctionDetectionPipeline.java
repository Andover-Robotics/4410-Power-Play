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
import org.opencv.core.Scalar;

/*
TODO
    NEED TO TUNE minwidth, yellowLowHSV, yellowHighHSV
 */

public class JunctionDetectionPipeline extends OpenCvPipeline{

    Telemetry telemetry;

    Mat HSV = new Mat();
    MatOfPoint biggest;

    public static int minwidth = 90; //try 45
    public static int width = 0;
    public static int camwidth = 1280;//try 640
    public static int camheight = 720;//try 360
    public enum JunctionVal{
        ONLEFT,
        ONRIGHT,
        ATJUNCTION,
        NOTDETECTED
    }
    public static JunctionVal junctionVal = JunctionVal.NOTDETECTED; // for monitoring our junction detection status; default is NOTDETECTED

    public static double lowH = 21, lowS = 112, lowV = 100, highH = 33, highS = 203, highV = 255;

    public static Scalar yellowLowHSV= new Scalar(lowH,lowS,lowV); // high and low yellow HSV values
    public static Scalar yellowHighHSV = new Scalar(highH,highS,highV); //grip says to use (21,160,50) low and (33, 255, 255) high


    public JunctionDetectionPipeline(Telemetry tele){ // constructor to get telemetry object
        telemetry = tele;
    }

    @Override
    public Mat processFrame(Mat input) {
        Imgproc.cvtColor(input, HSV, Imgproc.COLOR_RGB2HSV); //converting RGB colors to HSV

        Rect rightrect = new Rect(853, 1, 427, 719); // try 426, 1, 214, 359
        Rect leftrect = new Rect(1, 1, 577, 719); // rectangle sizes //try 1, 1, 288, 359

        Imgproc.rectangle(input, leftrect, new Scalar(255, 0, 0), 5); //displays rectangles with red color
        Imgproc.rectangle(input, rightrect, new Scalar(255, 0, 0), 5);

        // filters HSV mat into image with black being the lowest yellow HSV and white being the highest yellow HSV
        Core.inRange(HSV, yellowLowHSV, yellowHighHSV, HSV); // makes it easier to find contours

        List<MatOfPoint> contours = new ArrayList<>();
        Imgproc.findContours(HSV, contours, new Mat(), 0,1); // finds contours in HSV mat




        if (!contours.isEmpty()) { // checks if no contours are found
            contours.sort(Collections.reverseOrder(Comparator.comparingDouble(m -> Imgproc.boundingRect(m).width))); //orders contours in array from big to small(by width)
            // ____biggest are variables for the biggest on each side
            biggest = contours.get(0); //contour with the largest width(first in the array)
            // use biggest.width to get the width

            Rect rect = Imgproc.boundingRect(biggest); // turns biggest contour into a rectangle

            if (rect.width > minwidth) { // rectangle is bigger than 10 pixels
                Imgproc.rectangle(input, rect.tl(), rect.br(), new Scalar(0, 255, 0), 6); // puts border around contours with a green shade

                double midpointrect = rect.tl().x + rect.width/2.0; // gets midpoint x of the rectangle

                if (midpointrect > leftrect.tl().x && midpointrect < leftrect.br().x) { // checks if within boundaries of left side rectangle
                    junctionVal = JunctionVal.ONLEFT;
                } else if (midpointrect > rightrect.tl().x && midpointrect < rightrect.br().x) { // checks if within boundaries of right side rectangle
                    junctionVal = JunctionVal.ONRIGHT;
                } else if (midpointrect < rightrect.tl().x && midpointrect > leftrect.br().x){
                    junctionVal = JunctionVal.ATJUNCTION; // checks if in middle; means that it is scorable
                }
//
//                telemetry.addLine("Midpoint of Bounding Box :"+ midpointrect);
            } else {
                junctionVal = JunctionVal.NOTDETECTED;
            }
        } else {
            junctionVal = JunctionVal.NOTDETECTED;
        }
//        telemetry.addData("contours: ", contours.size());
//         telemetry.addData("Junction status: ",junctionVal);   is in test opmode

        // Releasing all our mats for the next iteration
        HSV.release();

        return input; // return end frame with rectangles drawn
    }
}