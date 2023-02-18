package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.auto.JunctionDetectionPipeline;
import org.firstinspires.ftc.teamcode.auto.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.teleop.subsystems.Bot;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;


@TeleOp(name = "MainTeleOp", group = "Competition")
public class MainTeleOp extends LinearOpMode {

    private Bot bot;
    private double driveSpeed = 1;
    private boolean isManual = false;


    private boolean leftTriggerPressed = false;
    private boolean rightTriggerPressed = false;





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

        bot = Bot.getInstance(this);
        GamepadEx gp2 = new GamepadEx(gamepad2);
        GamepadEx gp1 = new GamepadEx(gamepad1);
//        // Retrieve the IMU from the hardware map
//        BNO055IMU imu = hardwareMap.get(BNO055IMU.class, "imu");
//        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
//        // Technically this is the default, however specifying it is clearer
//        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
//        // Without this, data retrieving from the IMU throws an exception
//        imu.initialize(parameters);
        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {
            gp1.readButtons();
            gp2.readButtons();

            if (gp2.wasJustPressed(GamepadKeys.Button.Y)) {
                bot.claw.open();
            }else if(gp2.getButton(GamepadKeys.Button.X)) {
                bot.claw.close();
            }else if (bot.claw.getDistance() < 85) {
                if (gp2.getButton(GamepadKeys.Button.Y)) {
                    bot.claw.open();
                }else if (gp2.getButton(GamepadKeys.Button.A)) {
                    bot.claw.close();
                }
            }

            if (gp2.wasJustPressed(GamepadKeys.Button.DPAD_UP)) {
                bot.slide.runToTop();
            } else if (gp2.wasJustPressed(GamepadKeys.Button.DPAD_LEFT)) {
                bot.slide.runToMiddle();
            }else if (gp2.wasJustPressed(GamepadKeys.Button.DPAD_RIGHT)) {
                bot.slide.runToLow();
            }else if (gp2.wasJustPressed(GamepadKeys.Button.DPAD_DOWN)) {
                bot.slide.runToBottom();
            }
            if(gp2.wasJustPressed(GamepadKeys.Button.B)){
                while(junctionDetectionPipeline.getJunctionVal()!= JunctionDetectionPipeline.JunctionVal.AT_JUNCTION) {
                    while (junctionDetectionPipeline.getJunctionVal() != JunctionDetectionPipeline.JunctionVal.AT_JUNCTION) {
                        bot.rr.turn(0.001);
                        sleep(10);
                        telemetry.addData("JunctionDetection", junctionDetectionPipeline.getYellowPercentage());
                    }
                }
            }

            if (gp2.wasJustPressed(GamepadKeys.Button.RIGHT_BUMPER)) {
                bot.slide.goDown();
            }

            if (gp2.wasJustPressed(GamepadKeys.Button.LEFT_BUMPER)) {
                bot.slide.goUp();
            }

            if (gp2.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.9 && !rightTriggerPressed){
                rightTriggerPressed = true;
                bot.slide.goUpConeStack();
            }else if(gp2.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0.9 && !leftTriggerPressed){
                leftTriggerPressed = true;
                bot.slide.goDownConeStack();
            }
            if(gp2.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) < 0.7){
                leftTriggerPressed = false;
            }
            if(gp2.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) < 0.7){
                rightTriggerPressed = false;
            }

            bot.slide.periodic();


            // bot.turret.rotate(gp2.getLeftX());

            //TODO test sensor
            telemetry.addData("sensor", bot.claw.getDistance());
            telemetry.addData("drive current", bot.getCurrent());
            telemetry.addData("slide current", bot.slide.getCurrent());
            telemetry.update();

            FtcDashboard dash = FtcDashboard.getInstance();
            TelemetryPacket packet = new TelemetryPacket();
            packet.put("strafeSpeed", gp1.getRightX());
            packet.put("forwardSpeed", gp1.getLeftY());
            packet.put("turnSpeed", -gp1.getLeftX());
            dash.sendTelemetryPacket(packet);

            driveSpeed = 0.6;
            driveSpeed -= bot.slide.isHigh()/3;
            driveSpeed *= 1 - 0.5 * gp1.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER);
            driveSpeed = Math.max(0, driveSpeed);
            bot.fixMotors();
            bot.drive(gp1.getLeftX() * driveSpeed, -gp1.getLeftY() * driveSpeed, gp1.getRightX() * driveSpeed/1.5);
            }
        }
    }


