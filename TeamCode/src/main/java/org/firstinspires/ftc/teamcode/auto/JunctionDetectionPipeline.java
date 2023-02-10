package org.firstinspires.ftc.teamcode.auto;
import org.opencv.calib3d.Calib3d;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.Core;
import org.opencv.core.Rect;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core; // works with eocv-sim
import org.opencv.core.Mat; // also works
import org.opencv.core.Scalar; // also works
import org.opencv.core.MatOfDouble;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.MatOfPoint3f;
import org.opencv.core.Point;
import org.opencv.core.Point3;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.apriltag.AprilTagDetectorJNI;
import org.openftc.easyopencv.OpenCvPipeline;
import java.util.ArrayList;

public class JunctionDetectionPipeline extends OpenCvPipeline{
    Telemetry telemetry;
    public static double yellowPercentage;
    public static int horizOffset = 0, vertOffset = 0, width = 50, height = 50;
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
    Scalar yellowLowHSV= new Scalar(0,55,108);
    Scalar yellowHighHSV = new Scalar(54,209,255);
    //do not know yellow vals

    Mat smallMat = new Mat();


    @Override
    public Mat processFrame(Mat input) {
        //176 x 144= matrix size
        Imgproc.cvtColor(input, input, Imgproc.COLOR_RGB2HSV);
        ROI.x = input.width()/2 + horizOffset - 25;
        ROI.y = input.height()/2 + vertOffset - 25;
        ROI.width = width;
        ROI.height = height;
        smallMat = input.submat(ROI);
        Mat yellowMat = new Mat(176, 144, input.type());
        Core.inRange(smallMat, yellowLowHSV, yellowHighHSV, yellowMat);

        yellowPercentage = (Core.sumElems(yellowMat)).val[0]/25344;
        yellowMat.release();
        smallMat.release();

        if(yellowPercentage>=60){
            junctionVal = junctionVal.CLOSE_TO;
            telemetry.addData("Junction is approaching", yellowPercentage);
        }else if(yellowPercentage>=80){
            junctionVal = junctionVal.CLOSER_TO;
            telemetry.addData("Junction is closer", yellowPercentage);
        }else if(yellowPercentage>=90){
            junctionVal = junctionVal.AT_JUNCTION;
            telemetry.addData("At junction", yellowPercentage);
        }else{
            telemetry.addData("Junction has not been detected", yellowPercentage);
        }
        input.release();

        return input;

    }
    public JunctionVal getJunctionVal(){
        return junctionVal;
    }

    public double getYellowPercentage(){
        return yellowPercentage;
    }

    public double getROIx() { return ROI.x; }

    public double getROIy() { return ROI.y; }
}