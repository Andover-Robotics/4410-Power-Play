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
    private SignalVal signalVal = SignalVal.PINK;
    static Rect ROI = new Rect();

    public TestPipeline(Telemetry telemetry){
        this.telemetry = telemetry;
    }

    Scalar greenLowHSV= new Scalar(68,32,60);
    Scalar greenHighHSV = new Scalar(142,142,222);
    Scalar yellowLowHSV= new Scalar(0,0,117);
    Scalar yellowHighHSV = new Scalar(72,194,229);
    Scalar pinkLowHSV= new Scalar(134,73,80);
    Scalar pinkHighHSV = new Scalar(180,225,246);

    Mat greenMat = new Mat();
    Mat yellowMat = new Mat();
    Mat pinkMat = new Mat();
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

        Core.inRange(smallMat,greenLowHSV,greenHighHSV, greenMat);

        greenPercentage = (Core.sumElems(greenMat)).val[0];

        Core.inRange(smallMat, yellowLowHSV, yellowHighHSV, yellowMat);

        yellowPercentage = (Core.sumElems(yellowMat)).val[0];

        Core.inRange(smallMat,pinkLowHSV,pinkHighHSV,pinkMat);

        pinkPercentage = (Core.sumElems(pinkMat)).val[0];

        pinkMat.release();
        yellowMat.release();
        greenMat.release();
        smallMat.release();

        //Math.round((Core.sumElems(epicSubmat).val[0]/ROI.area()/255)*100);


       // input.release();

        double greatestNum = Math.max(Math.max(greenPercentage,pinkPercentage), yellowPercentage);

        if(greatestNum == greenPercentage){
            signalVal = SignalVal.GREEN;
//            telemetry.addData("The signal value is green with a percentage of", greenPercentage);
//            telemetry.update();
        }else if(greatestNum == yellowPercentage){
            signalVal = SignalVal.YELLOW;
//            telemetry.addData("The signal value is yellow with a percentage of",yellowPercentage);
//            telemetry.update();
        }else if(greatestNum == pinkPercentage){
            signalVal = SignalVal.PINK;
//            telemetry.addData("The signal value is red with a percentage of",pinkPercentage);
//            telemetry.update();
        }
//        telemetry.addData("Green is",greenPercentage);
//        telemetry.addData("Yellow is",yellowPercentage);
//        telemetry.addData("Pink is",pinkPercentage);
//        telemetry.addData("Matrix size",input.size());
//        telemetry.addData("I think it's", getSignalVal().toString());
//        telemetry.update();

        input.release();
//        Imgproc.rectangle(input, ROI, new Scalar(255, 255, 255));
//        Imgproc.cvtColor(input, input, Imgproc.COLOR_HSV2RGB);

        return input;


    }
    public SignalVal getSignalVal(){
        return signalVal;
    }
}


