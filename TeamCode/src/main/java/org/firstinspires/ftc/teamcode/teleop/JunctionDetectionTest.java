package org.firstinspires.ftc.teamcode.teleop;


import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import com.arcrobotics.ftclib.gamepad.GamepadEx;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.teleop.subsystems.Bot;
import org.firstinspires.ftc.teamcode.teleop.subsystems.JunctionDetectionPipeline;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Autonomous(name = "Test Junction Detection", group = "Test")
public class JunctionDetectionTest extends LinearOpMode {

    private Bot bot;

    @Override
    public void runOpMode() throws InterruptedException {

        WebcamName camName = hardwareMap.get(WebcamName.class, "Webcam 1");
        OpenCvCamera camera = OpenCvCameraFactory.getInstance().createWebcam(camName);
        JunctionDetectionPipeline junctionDetectionPipeline = new JunctionDetectionPipeline(telemetry);
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

            //bot.turretalignjunction();

            //bot.slidesalignjunction();

            telemetry.addData("Junction Status: ", JunctionDetectionPipeline.junctionVal);
            telemetry.addData("Low: ", JunctionDetectionPipeline.yellowLowHSV);
            telemetry.addData("High: ", JunctionDetectionPipeline.yellowHighHSV);
            telemetry.addData("minwidth = ", JunctionDetectionPipeline.minwidth);
            telemetry.addLine("UP/DOWN for highH, LEFT/RIGHT for lowH");
            telemetry.addLine("Y/A for highS, X/B for lowS");
            telemetry.addLine("Triggers for Minwidth +/- 5");



            if (gp1.wasJustPressed(GamepadKeys.Button.DPAD_UP)) {
                JunctionDetectionPipeline.highH += 1;
            }
            if (gp1.wasJustPressed(GamepadKeys.Button.DPAD_DOWN)) {
                JunctionDetectionPipeline.highH -= 1;
            }
            if (gp1.wasJustPressed(GamepadKeys.Button.DPAD_LEFT)) {
                JunctionDetectionPipeline.lowH -= 1;
            }
            if (gp1.wasJustPressed(GamepadKeys.Button.DPAD_RIGHT)) {
                JunctionDetectionPipeline.lowH += 1;
            }
            if (gp1.wasJustPressed(GamepadKeys.Button.Y)) {
                JunctionDetectionPipeline.highS += 1;
            }
            if (gp1.wasJustPressed(GamepadKeys.Button.A)) {
                JunctionDetectionPipeline.highS -= 1;
            }
            if (gp1.wasJustPressed(GamepadKeys.Button.X)) {
                JunctionDetectionPipeline.lowS -= 1;
            }
            if (gp1.wasJustPressed(GamepadKeys.Button.B)) {
                JunctionDetectionPipeline.lowS += 1;
            }
            if (gp1.wasJustPressed(GamepadKeys.Button.LEFT_BUMPER)) {
                JunctionDetectionPipeline.minwidth -= 5;
            }
            if (gp1.wasJustPressed(GamepadKeys.Button.RIGHT_BUMPER)) {
                JunctionDetectionPipeline.minwidth += 5;
            }
            telemetry.update();







        }
        while (!isStarted()) {
            telemetry.addData("Junction Status: ", JunctionDetectionPipeline.junctionVal);
            telemetry.addData("Low: ", JunctionDetectionPipeline.yellowLowHSV);
            telemetry.addData("High: ", JunctionDetectionPipeline.yellowHighHSV);
            telemetry.addData("minwidth = ", JunctionDetectionPipeline.minwidth);
            telemetry.addLine("UP/DOWN for highH, LEFT/RIGHT for lowH");
            telemetry.addLine("Y/A for highS, X/B for lowS");
            telemetry.addLine("Triggers for Minwidth +/- 5");

            if (gp1.wasJustPressed(GamepadKeys.Button.DPAD_UP)) {
                JunctionDetectionPipeline.highH += 1;
            }
            if (gp1.wasJustPressed(GamepadKeys.Button.DPAD_DOWN)) {
                JunctionDetectionPipeline.highH -= 1;
            }
            if (gp1.wasJustPressed(GamepadKeys.Button.DPAD_LEFT)) {
                JunctionDetectionPipeline.lowH -= 1;
            }
            if (gp1.wasJustPressed(GamepadKeys.Button.DPAD_RIGHT)) {
                JunctionDetectionPipeline.lowH += 1;
            }
            if (gp1.wasJustPressed(GamepadKeys.Button.Y)) {
                JunctionDetectionPipeline.highS += 1;
            }
            if (gp1.wasJustPressed(GamepadKeys.Button.A)) {
                JunctionDetectionPipeline.highS -= 1;
            }
            if (gp1.wasJustPressed(GamepadKeys.Button.X)) {
                JunctionDetectionPipeline.lowS -= 1;
            }
            if (gp1.wasJustPressed(GamepadKeys.Button.B)) {
                JunctionDetectionPipeline.lowS += 1;
            }
            if (gp1.wasJustPressed(GamepadKeys.Button.LEFT_BUMPER)) {
                JunctionDetectionPipeline.minwidth -= 5;
            }
            if (gp1.wasJustPressed(GamepadKeys.Button.RIGHT_BUMPER)) {
                JunctionDetectionPipeline.minwidth += 5;
            }
            telemetry.update();
        }

        if (isStarted()) {
//            camera.stopStreaming();
            camera.closeCameraDevice();
        }




    }
}