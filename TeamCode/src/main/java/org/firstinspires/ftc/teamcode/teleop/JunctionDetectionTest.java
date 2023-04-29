package org.firstinspires.ftc.teamcode.teleop;


import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import com.arcrobotics.ftclib.gamepad.GamepadEx;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.teleop.subsystems.v2junctiondetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Autonomous(name = "Test Junction Detection", group = "Test")
public class JunctionDetectionTest extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        WebcamName camName = hardwareMap.get(WebcamName.class, "Webcam 1");
        OpenCvCamera camera = OpenCvCameraFactory.getInstance().createWebcam(camName);
        v2junctiondetection junctionDetectionPipeline = new v2junctiondetection(telemetry);
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
        camera.setPipeline(junctionDetectionPipeline);

        GamepadEx gp1 = new GamepadEx(gamepad1);
        GamepadEx gp2 = new GamepadEx(gamepad2);
        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {


            telemetry.addData("Junction Detected: ", v2junctiondetection.junctionVal);
            telemetry.addData("Low: ", v2junctiondetection.yellowLowHSV);
            telemetry.addData("High: ", v2junctiondetection.yellowHighHSV);
            telemetry.addData("minwidth = ", v2junctiondetection.minwidth);
            telemetry.addLine("UP/DOWN for highH, LEFT/RIGHT for lowH");
            telemetry.addLine("Y/A for highS, X/B for lowS");
            telemetry.addLine("Triggers for Minwidth +/- 5");

            if (gp1.wasJustPressed(GamepadKeys.Button.DPAD_UP)) {
                v2junctiondetection.highH += 1;
            }
            if (gp1.wasJustPressed(GamepadKeys.Button.DPAD_DOWN)) {
                v2junctiondetection.highH -= 1;
            }
            if (gp1.wasJustPressed(GamepadKeys.Button.DPAD_LEFT)) {
                v2junctiondetection.lowH -= 1;
            }
            if (gp1.wasJustPressed(GamepadKeys.Button.DPAD_RIGHT)) {
                v2junctiondetection.lowH += 1;
            }
            if (gp1.wasJustPressed(GamepadKeys.Button.Y)) {
                v2junctiondetection.highS += 1;
            }
            if (gp1.wasJustPressed(GamepadKeys.Button.A)) {
                v2junctiondetection.highS -= 1;
            }
            if (gp1.wasJustPressed(GamepadKeys.Button.X)) {
                v2junctiondetection.lowS -= 1;
            }
            if (gp1.wasJustPressed(GamepadKeys.Button.B)) {
                v2junctiondetection.lowS += 1;
            }
            if (gp1.wasJustPressed(GamepadKeys.Button.LEFT_BUMPER)) {
                v2junctiondetection.minwidth -= 5;
            }
            if (gp1.wasJustPressed(GamepadKeys.Button.RIGHT_BUMPER)) {
                v2junctiondetection.minwidth += 5;
            }
            telemetry.update();







        }
        while (!isStarted()) {
            telemetry.addData("Junction Status: ", v2junctiondetection.junctionVal);
            telemetry.update();
        }

        if (isStarted()) {
//            camera.stopStreaming();
            camera.closeCameraDevice();
        }




    }
}