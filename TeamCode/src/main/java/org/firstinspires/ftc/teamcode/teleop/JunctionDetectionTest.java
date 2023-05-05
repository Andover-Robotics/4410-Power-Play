package org.firstinspires.ftc.teamcode.teleop;


import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.gamepad.TriggerReader;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import com.arcrobotics.ftclib.gamepad.GamepadEx;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.teleop.subsystems.Bot;
import org.firstinspires.ftc.teamcode.teleop.subsystems.JunctionDetectionPipeline;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@TeleOp(name = "Test Junction Detection", group = "Test")
public class JunctionDetectionTest extends LinearOpMode {

    private Bot bot;
    private double cycleTime = 1;

    @Override
    public void runOpMode() throws InterruptedException {

        WebcamName camName = hardwareMap.get(WebcamName.class, "Webcam 1");
        OpenCvCamera camera = OpenCvCameraFactory.getInstance().createWebcam(camName);
        JunctionDetectionPipeline junctionDetectionPipeline = new JunctionDetectionPipeline(telemetry);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(1280, 720, OpenCvCameraRotation.UPRIGHT); //try 640 and 360
            }

            @Override
            public void onError(int errorCode) {
                telemetry.addData("Error code:", errorCode);
            }
        });
        camera.setPipeline(junctionDetectionPipeline);

        bot = Bot.getInstance(this);

        GamepadEx gp1 = new GamepadEx(gamepad1);
        GamepadEx gp2 = new GamepadEx(gamepad2);


        bot.initializeImus();

        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {

            telemetry.addData("Junction Status: ", JunctionDetectionPipeline.junctionVal);
            telemetry.addData("Low: ", JunctionDetectionPipeline.yellowLowHSV);
            telemetry.addData("High: ", JunctionDetectionPipeline.yellowHighHSV);
            telemetry.addData("minwidth = ", JunctionDetectionPipeline.minwidth);
            telemetry.addData("width: ", JunctionDetectionPipeline.width);

            bot.turretalignjunction();

            telemetry.addData("cycle", time - cycleTime);
            cycleTime = time;

            telemetry.update();
            bot.turret.periodic();

        }
        while (!isStarted()) {
            telemetry.addData("Junction Status: ", JunctionDetectionPipeline.junctionVal);
            telemetry.addData("Low: ", JunctionDetectionPipeline.yellowLowHSV);
            telemetry.addData("High: ", JunctionDetectionPipeline.yellowHighHSV);
            telemetry.addData("minwidth = ", JunctionDetectionPipeline.minwidth);
            telemetry.addData("width: ", JunctionDetectionPipeline.width);


            telemetry.update();

        }



        if (isStarted()) {
//            camera.stopStreaming();
            camera.closeCameraDevice();
        }

    }
}