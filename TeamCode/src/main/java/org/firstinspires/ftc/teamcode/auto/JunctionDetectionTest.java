package org.firstinspires.ftc.teamcode.auto;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.teleop.subsystems.Bot;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;


@Autonomous(name = "Test Junction Detection", group = "Test")
public class JunctionDetectionTest extends LinearOpMode {

    Bot bot;

    @Override
    public void runOpMode() throws InterruptedException {


        double percent;
        double sumPercent = 0;
        double average = 0;

        JunctionDetectionPipeline.JunctionVal junctionVal;

        WebcamName camName = hardwareMap.get(WebcamName.class, "Webcam 1");
        OpenCvCamera camera = OpenCvCameraFactory.getInstance().createWebcam(camName);
        JunctionDetectionPipeline junctionDetectionPipeline = new JunctionDetectionPipeline(telemetry);
        camera.setPipeline(junctionDetectionPipeline);

        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(1280, 720, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
                telemetry.addData("Error code:", errorCode);
            }
        });

        while (!isStarted()){

            for(int i = 1; i < 1000000; i++) {
            percent = junctionDetectionPipeline.getYellowPercentage();
            sumPercent += percent;
            average = sumPercent/i;
            junctionVal = junctionDetectionPipeline.getJunctionVal(); }

            while(junctionDetectionPipeline.getJunctionVal() == JunctionDetectionPipeline.JunctionVal.NOT_DETECTED || junctionDetectionPipeline.getJunctionVal() == JunctionDetectionPipeline.JunctionVal.AT_JUNCTION) {

                telemetry.addData("Junction Status", junctionDetectionPipeline.getJunctionVal());
                telemetry.addData("Yellow Percentage", junctionDetectionPipeline.getYellowPercentage());
                telemetry.addData("Average yellow percentage", average);
                telemetry.update();
            }

            while(junctionDetectionPipeline.getJunctionVal() == JunctionDetectionPipeline.JunctionVal.CLOSE_TO || junctionDetectionPipeline.getJunctionVal() == JunctionDetectionPipeline.JunctionVal.CLOSER_TO)
            {
                for(int i = 1; i <= 10000; i++) {
                    percent = junctionDetectionPipeline.getYellowPercentage();
                    sumPercent += percent;
                    average = sumPercent/i;
                    junctionVal = junctionDetectionPipeline.getJunctionVal();

                }

                percent = average;
                junctionVal = junctionDetectionPipeline.getJunctionValue(percent);

                telemetry.addData("Junction Status", junctionVal);
                telemetry.addData("Yellow Percentage", percent);
                telemetry.update();
            }

        }
        if(isStarted()){
            camera.stopStreaming();
            camera.closeCameraDevice();
        }
    }
}
