package org.firstinspires.ftc.teamcode.teleop.subsystems;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.teleop.subsystems.Bot;
import org.firstinspires.ftc.teamcode.teleop.subsystems.JunctionDetectionPipeline;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Autonomous(name = "Test Junction Detection", group = "Test")
public class JunctionDetectionTest extends LinearOpMode {

    Bot bot;

    @Override
    public void runOpMode() throws InterruptedException {

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
            telemetry.addData("Junction Status: ", junctionDetectionPipeline.junctionVal);
            telemetry.update();
        }

        if(isStarted()){
            camera.stopStreaming();
            camera.closeCameraDevice();
        }
    }
}