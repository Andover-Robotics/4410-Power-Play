package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.teleop.subsystems.Bot;
import org.firstinspires.ftc.teamcode.teleop.subsystems.JunctionDetectionPipeline;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraException;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;
import java.util.Map;


@Config
@Autonomous(name = "MainAutonomous")
public class MainAutonomous extends LinearOpMode {

    Bot bot;

    private double moveDiff = -1;
    private boolean autoaim = false;

    enum Side {
        RIGHT, LEFT, NULL;
    }

    Side side = Side.NULL;

    boolean isTestMode = false;

    public static int driveTime = 2000, timeSlidesUp = 900, timeSlidesDown = 550, timeOuttake = 350, timeConeDrop = 150, timeIntakeDown = 200, timeIntakeOut = 700, timeIntakeClose = 350, timeIntakeUp = 450, timeIntakeIn = 400;//old 400

    //    private static int horizIntake = {}
    static final double FEET_PER_METER = 3.28084;

    double fx = 1078.03779;
    double fy = 1084.50988;
    double cx = 580.850545;
    double cy = 245.959325;

    // UNITS ARE METERS
    double tagsize = 0.032; //ONLY FOR TESTING

    // Tag ID 1,2,3 from the 36h11 family
    int ID_ONE = 1;
    int ID_TWO = 2;
    int ID_THREE = 3;
    AprilTagDetection tagOfInterest = null;

    @Override
    public void runOpMode() throws InterruptedException {

        telemetry.setAutoClear(true);
        bot = Bot.getInstance(this);
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        GamepadEx gp1 = new GamepadEx(gamepad1);
        GamepadEx gp2 = new GamepadEx(gamepad2);
        bot.arm.preload();

        //CAMERA STUFF =====================

        WebcamName camName = hardwareMap.get(WebcamName.class, "Webcam 1");
        OpenCvCamera camera = OpenCvCameraFactory.getInstance().createWebcam(camName);
        AprilTagDetectionPipeline aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);

        camera.setPipeline(aprilTagDetectionPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(1280, 720, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {

            }
        });


        Pose2d startPose = new Pose2d(0, 0, 0);
        drive.setPoseEstimate(startPose);


        Thread periodic = new Thread(() -> {
            while (opModeIsActive() && !isStopRequested()) {
                bot.slides.periodic();
                bot.turret.periodic();
                bot.horizSlides.periodic();
            }
        });

//        telemetry.setMsTransmissionInterval(50);


        while (!isStarted()) {
            gp1.readButtons();
            if (gp1.wasJustPressed(GamepadKeys.Button.B)) {
                side = Side.RIGHT;
            }
            if (gp1.wasJustPressed(GamepadKeys.Button.X)) {
                side = Side.LEFT;
            }

            if (gp1.wasJustPressed(GamepadKeys.Button.A)) {
                bot.claw.close();
            }else if(gp1.wasJustPressed(GamepadKeys.Button.START)){
                bot.claw.open();
            }
            if (gp1.wasJustPressed(GamepadKeys.Button.BACK)) {
                isTestMode = true;
            }
            if (gp1.wasJustPressed(GamepadKeys.Button.LEFT_STICK_BUTTON)) {
                autoaim = true;
            }
            if (gp1.wasJustPressed(GamepadKeys.Button.RIGHT_STICK_BUTTON)) {
                autoaim = false;
            }
            telemetry.addData("moveDiff (positive is more ???)", moveDiff);
            if(gp1.wasJustPressed(GamepadKeys.Button.LEFT_BUMPER)){
                moveDiff -= 0.5;
            }else if(gp1.wasJustPressed(GamepadKeys.Button.RIGHT_BUMPER)){
                moveDiff += 0.5;
            }
            if(side == Side.LEFT){
                telemetry.addData("turretOuttakeLeft", bot.turret.turretAutoOuttakeLeft);
                if(gp1.wasJustPressed(GamepadKeys.Button.DPAD_LEFT)){
                    bot.turret.turretAutoOuttakeLeft -= 5;
                }else if(gp1.wasJustPressed(GamepadKeys.Button.DPAD_RIGHT)){
                    bot.turret.turretAutoOuttakeLeft += 5;
                }
            }else if(side == Side.RIGHT) {
                telemetry.addData("turretOuttakeRight", bot.turret.turretAutoOuttakeRight);
                if(gp1.wasJustPressed(GamepadKeys.Button.DPAD_LEFT)){
                    bot.turret.turretAutoOuttakeRight -= 5;
                }else if(gp1.wasJustPressed(GamepadKeys.Button.DPAD_RIGHT)){
                    bot.turret.turretAutoOuttakeRight += 5;
                }
            }

            telemetry.addData("side?", side.toString());
            telemetry.addData("testmode", isTestMode);
            telemetry.addData("AutoAim", autoaim);

            telemetry.addData("Current FPS:", camera.getFps());
            telemetry.addData("Current Max FPS:", camera.getCurrentPipelineMaxFps());

            ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();

            if (currentDetections.size() != 0) {
                boolean tagFound = false;

                for (AprilTagDetection tag : currentDetections) {
                    if (tag.id == ID_ONE || tag.id == ID_TWO || tag.id == ID_THREE) {
                        tagOfInterest = tag;
                        tagFound = true;
                        break;
                    }
                }

                if (tagFound) {
                    telemetry.addLine("Tag of interest is in sight!\n\nLocation data:");
                    tagToTelemetry(tagOfInterest);
                } else {
                    telemetry.addLine("Don't see tag of interest :(");

                    if (tagOfInterest == null) {
                        telemetry.addLine("(The tag has never been seen)");
                    } else {
                        telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                        tagToTelemetry(tagOfInterest);
                    }
                }

            } else {
                telemetry.addLine("Don't see tag of interest :(");

                if (tagOfInterest == null) {
                    telemetry.addLine("(The tag has never been seen)");
                } else {
                    telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                    tagToTelemetry(tagOfInterest);
                }

            }

            telemetry.update();
            sleep(20);

        }

        try {
            camera.stopStreaming();
            camera.closeCameraDevice();
        } catch (OpenCvCameraException e) {

        }
        //END CAMERA STUFF ===============

        Trajectory forward = drive.trajectoryBuilder(startPose)
                .lineTo(new Vector2d(52, moveDiff))
                .build();

        Trajectory parkLeft = drive.trajectoryBuilder(forward.end())
                .strafeLeft(22)
                .build();
        Trajectory parkRight = drive.trajectoryBuilder(forward.end())
                .strafeRight(24)
                .build();

        Thread driveForward = new Thread(() -> drive.followTrajectory(forward));
        bot.resetIMU();

        waitForStart();
        if (!isStopRequested()) {
            bot.claw.close();
            bot.arm.autoStorage();

            periodic.start();

            driveForward.start();
            sleep(driveTime);

            if (side != Side.NULL) {
                outtake(5);
                for (int i = 4; i >= 0; i--) {
                    telemetry.addData("running cycle", i);
                    telemetry.update();
                    pickUpCone(i);
                    outtake(i);
                }
            } else {
                sleep(3000);
            }

            if (isTestMode) {
                Trajectory goBack = drive.trajectoryBuilder(forward.end())
                        .lineTo(new Vector2d())
                        .build();
                drive.followTrajectory(goBack);
            } else {
                try {

                    if (tagOfInterest.id == ID_ONE) {
                        drive.followTrajectory(parkLeft);
                    } else if (tagOfInterest.id == ID_THREE) {
                        drive.followTrajectory(parkRight);
                    } else {
                        sleep(1500);
                    }
                } catch (NullPointerException e) {
                    sleep(1500);
                }
            }
            driveForward.interrupt();
            periodic.interrupt();
        }
    }

    private void outtake(int i) {
        if (side == Side.RIGHT) {
            bot.turret.runToAutoOuttakeRight(bot.getIMU());
        } else {
            bot.turret.runToAutoOuttakeLeft(bot.getIMU());
        }
        //old ====
        bot.slides.runToTop();
        sleep(timeSlidesUp);
        if(i == 5){
            sleep(400);
        }
        bot.arm.autoOuttake();
        sleep(timeOuttake);
        bot.arm.autoSecure();
        //end old, new =====
//        bot.slides.runToTop();
//        bot.arm.brace();
//        sleep(timeSlidesUp);
//        if (i == 5) {
//            sleep(400);
//        }
//        bot.horizSlides.runToAutoOuttake();
//        sleep(timeOuttake);
        //end new
        bot.claw.open();
        bot.arm.brace();
        sleep(timeConeDrop);
        bot.claw.close();
        sleep(timeOuttake);
        bot.slides.runToBottom();
        if (i > 0) {
            if (side == Side.RIGHT) {
                bot.turret.runToAutoIntakeRight(bot.getIMU());
            } else {
                bot.turret.runToAutoIntakeLeft(bot.getIMU());
            }
        } else {
            bot.turret.runToFront();
            bot.arm.preload();
        }
        sleep(timeSlidesDown);
    }

    private void pickUpCone(int i) {
        bot.claw.open();
        bot.arm.intakeAuto(i);
        sleep(timeIntakeDown);
        bot.horizSlides.runToAutoIntake();
        sleep(timeIntakeOut);
        bot.claw.close();
        sleep(timeIntakeClose);
        bot.slides.runToLow();
        bot.arm.autoStorage();
        if (i > 0) {
            sleep(timeIntakeUp);
        }
        bot.horizSlides.runToFullIn();
        sleep(timeIntakeIn);
    }


    void tagToTelemetry(AprilTagDetection detection) {
        telemetry.addLine(String.format("Detected tag ID=%d", detection.id));
        telemetry.addLine(String.format("Translation X: %.2f feet", detection.pose.x * FEET_PER_METER));
        telemetry.addLine(String.format("Translation Y: %.2f feet", detection.pose.y * FEET_PER_METER));
        telemetry.addLine(String.format("Translation Z: %.2f feet", detection.pose.z * FEET_PER_METER));
        telemetry.addLine(String.format("Rotation Yaw: %.2f degrees", Math.toDegrees(detection.pose.yaw)));
        telemetry.addLine(String.format("Rotation Pitch: %.2f degrees", Math.toDegrees(detection.pose.pitch)));
        telemetry.addLine(String.format("Rotation Roll: %.2f degrees", Math.toDegrees(detection.pose.roll)));
    }

}

