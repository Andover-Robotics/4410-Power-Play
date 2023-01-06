package org.firstinspires.ftc.teamcode.auto;

import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core; // works with eocv-sim
import org.opencv.core.Mat; // also works
import org.opencv.core.Scalar; // also works


public class TestPipeline extends OpenCvPipeline{
    Telemetry telemetry;
    public static double greenPercentage;
    public static double yellowPercentage;
    public static double pinkPercentage;
    public static int horizOffset = 0, vertOffset = 0, width = 50, height = 50;
    public enum SignalVal{
        GREEN,
        YELLOW,
        PINK
    }
    private SignalVal signalVal;
    static Rect ROI = new Rect();

    public TestPipeline(Telemetry telemetry){
        this.telemetry = telemetry;
    }

    Scalar greenLowHSV= new Scalar(68,21,30);
    Scalar greenHighHSV = new Scalar(113,203,179);
    Scalar yellowLowHSV= new Scalar(19,48,117);
    Scalar yellowHighHSV = new Scalar(119,134,229);
    Scalar pinkLowHSV= new Scalar(134,73,80);
    Scalar pinkHighHSV = new Scalar(180,225,246);

    @Override
    public Mat processFrame(Mat input) {
        //176 x 144= matrix size
        //

        Imgproc.cvtColor(input, input, Imgproc.COLOR_RGB2HSV);
        ROI.x = input.width()/2 + horizOffset - 25;
        ROI.y = input.height()/2 + vertOffset - 25;
        ROI.width = width;
        ROI.height = height;


        //mask: takes image and tells which pixels to copy and which to leave black



        Mat epicSubmat = input.submat(ROI);


        Core.inRange(input.submat(ROI),greenLowHSV,greenHighHSV, epicSubmat);


        greenPercentage = (Core.sumElems(epicSubmat)).val[0];


        Core.inRange(input.submat(ROI), yellowLowHSV, yellowHighHSV, epicSubmat);

        yellowPercentage = (Core.sumElems(epicSubmat)).val[0];

        Core.inRange(input.submat(ROI),pinkLowHSV,pinkHighHSV,epicSubmat);

        pinkPercentage = (Core.sumElems(epicSubmat)).val[0];

        //Math.round((Core.sumElems(epicSubmat).val[0]/ROI.area()/255)*100);


       // input.release();

        double greatestNum = Math.max(Math.max(greenPercentage,pinkPercentage), yellowPercentage);

        if(greatestNum == greenPercentage){
            signalVal = SignalVal.GREEN;
            telemetry.addData("The signal value is green with a percentage of", greenPercentage);
            telemetry.update();
        }else if(greatestNum == yellowPercentage){
            signalVal = SignalVal.YELLOW;
            telemetry.addData("The signal value is yellow with a percentage of",yellowPercentage);
            telemetry.update();
        }else if(greatestNum == pinkPercentage){
            signalVal = SignalVal.PINK;
            telemetry.addData("The signal value is red with a percentage of",pinkPercentage);
            telemetry.update();
        }
        telemetry.addData("Green is",greenPercentage);
        telemetry.addData("Yellow is",yellowPercentage);
        telemetry.addData("Pink is",pinkPercentage);
        telemetry.addData("Matrix size",input.size());
        telemetry.addData("I think it's", getSignalVal().toString());
        telemetry.update();

        Imgproc.rectangle(input, ROI, new Scalar(255, 255, 255));
        Imgproc.cvtColor(input, input, Imgproc.COLOR_HSV2RGB);

        return input;


    }
    public SignalVal getSignalVal(){
        return signalVal;
    }
}


