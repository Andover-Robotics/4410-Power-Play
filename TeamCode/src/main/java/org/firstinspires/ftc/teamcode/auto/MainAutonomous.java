package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.teleop.subsystems.Bot;
import org.firstinspires.ftc.teamcode.teleop.subsystems.Turret;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;


@Config
@Autonomous(name="MainAutonomous")
public class MainAutonomous extends LinearOpMode {

    Bot bot;
    boolean isRight;

    public static int timeSlidesUp = 800, timeSlidesDown = 550, timeOuttake = 350, timeConeDrop = 250, timeIntakeDown = 200, timeIntakeOut = 700, timeIntakeClose = 250, timeIntakeUp = 300, timeIntakeIn = 400;


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

        // Retrieve the IMU from the hardware map
//        BNO055IMU imu = hardwareMap.get(BNO055IMU.class, "imu");
//        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
//        // Technically this is the default, however specifying it is clearer
//        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
//        // Without this, data retrieving from the IMU throws an exception
//        imu.initialize(parameters);

        GamepadEx gp1 = new GamepadEx(gamepad1);
        GamepadEx gp2 = new GamepadEx(gamepad2);
        bot.arm.preload();
        while(!isStarted()){
            gp1.readButtons();
            if(gp1.wasJustPressed(GamepadKeys.Button.A)){
                bot.claw.close();
            }
        }

        //CAMERA STUFF =====================

//        WebcamName camName = hardwareMap.get(WebcamName.class, "Webcam 1");
//        OpenCvCamera camera = OpenCvCameraFactory.getInstance().createWebcam(camName);
//        AprilTagDetectionPipeline aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);
//
//        camera.setPipeline(aprilTagDetectionPipeline);
//        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
//        {
//            @Override
//            public void onOpened()
//            {
//                camera.startStreaming(1280,720, OpenCvCameraRotation.UPRIGHT);
//            }
//
//            @Override
//            public void onError(int errorCode)
//            {
//
//            }
//        });
//
//        telemetry.setMsTransmissionInterval(50);
//
//
//
//        while(!isStarted()) {
//            if(gp1.wasJustPressed(GamepadKeys.Button.X)){
//                isRight=!isRight;
//            }
//            telemetry.addData("Current FPS:", camera.getFps());
//            telemetry.addData("Current Max FPS:", camera.getCurrentPipelineMaxFps());
//            telemetry.addData("Is right side?", isRight);
//
//            ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();
//
//            if(currentDetections.size() != 0)
//            {
//                boolean tagFound = false;
//
//                for(AprilTagDetection tag : currentDetections)
//                {
//                    if(tag.id == ID_ONE || tag.id == ID_TWO || tag.id == ID_THREE)
//                    {
//                        tagOfInterest = tag;
//                        tagFound = true;
//                        break;
//                    }
//                }
//
//                if(tagFound)
//                {
//                    telemetry.addLine("Tag of interest is in sight!\n\nLocation data:");
//                    tagToTelemetry(tagOfInterest);
//                }
//                else
//                {
//                    telemetry.addLine("Don't see tag of interest :(");
//
//                    if(tagOfInterest == null)
//                    {
//                        telemetry.addLine("(The tag has never been seen)");
//                    }
//                    else
//                    {
//                        telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
//                        tagToTelemetry(tagOfInterest);
//                    }
//                }
//
//            }
//            else
//            {
//                telemetry.addLine("Don't see tag of interest :(");
//
//                if(tagOfInterest == null)
//                {
//                    telemetry.addLine("(The tag has never been seen)");
//                }
//                else
//                {
//                    telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
//                    tagToTelemetry(tagOfInterest);
//                }
//
//            }
//
//            telemetry.update();
//            sleep(20);
//
//            gp1.readButtons();
//            if(gp1.wasJustPressed(GamepadKeys.Button.Y)) {
//                bot.claw.close();
//            }
//
//            telemetry.update();
//
//        }
//
//        camera.stopStreaming();
//        camera.closeCameraDevice();

        //END CAMERA STUFF ===============

        waitForStart();

//        bot.claw.close();
//
//        Thread slidePeriodic = new Thread(() -> {
//            while(opModeIsActive()){
//                bot.slides.periodic();
//            }
//        });
//        slidePeriodic.start();
//        bot.claw.close();
//
//
//
//        Pose2d startPose = new Pose2d(0,0,0);
//        Pose2d park1Pose = new Pose2d(12, 0,0);
//        Pose2d park2Pose = new Pose2d(12, 12, 0);
//
//
//        Trajectory forward = bot.rr.trajectoryBuilder(startPose)
//                .forward(48)
//                .build();
//
//
//        //Alliance 1 trajectories
//        Trajectory alliance1StrafeRight = bot.rr.trajectoryBuilder(forward.end())
//                .strafeRight(13)
//                .build();
//        Trajectory alliance1ApproachJunction = bot.rr.trajectoryBuilder(alliance1StrafeRight.end())
//                .forward(6)
//                .build();
//        Trajectory alliance1GoBack = bot.rr.trajectoryBuilder(alliance1ApproachJunction.end())
//                .back(4)
//                .build();
//        Trajectory alliance1GoToCone = bot.rr.trajectoryBuilder(alliance1GoBack.end())
//                .strafeLeft(28)
//                .build();
//        Trajectory alliance1GoForwardToCone = bot.rr.trajectoryBuilder(new Pose2d(alliance1GoToCone.end().getX(), alliance1GoToCone.end().getY(), Math.toRadians(90)))
//                .forward(4)
//                .build();
//        Trajectory alliance1GoToJunction = bot.rr.trajectoryBuilder(new Pose2d(alliance1GoForwardToCone.end().getX(), alliance1GoForwardToCone.end().getY(), Math.toRadians(0)))
//                .strafeRight(32)
//                .build();
//
//       /* Trajectory goBack = bot.rr.trajectoryBuilder(new Pose2d(allianceOneGoToJunction.end().getX(), allianceOneGoToJunction.end().getY(), -Math.toRadians(90)))
//                .back(6)
//                .build();
//        */
//
//        //Alliance 2 trajectories
//        Trajectory alliance2StrafeLeft= bot.rr.trajectoryBuilder(forward.end())
//                .strafeLeft(13)
//                .build();
//        Trajectory alliance2ApproachJunction= bot.rr.trajectoryBuilder(alliance2StrafeLeft.end())
//                .forward(6)
//                .build();
//        Trajectory alliance2GoBack = bot.rr.trajectoryBuilder(alliance2ApproachJunction.end())
//                .back(4)
//                .build();
//        Trajectory alliance2GoToCone = bot.rr.trajectoryBuilder(alliance2GoBack.end())
//                .strafeRight(28)
//                .build();
//        Trajectory alliance2GoForwardToCone = bot.rr.trajectoryBuilder(new Pose2d(alliance2GoToCone.end().getX(), alliance2GoToCone.end().getY(), -Math.toRadians(90)))
//                .forward(4)
//                .build();
//        Trajectory alliance2GoToJunction = bot.rr.trajectoryBuilder(new Pose2d(alliance2GoForwardToCone.end().getX(), alliance2GoForwardToCone.end().getY(), Math.toRadians(0)))
//                .strafeLeft(32)
//                .build();
//
//
//        bot.rr.followTrajectory(forward);
//
//        if(!isRight) {
//            bot.slides.runToTop();
//            bot.rr.followTrajectory(alliance1StrafeRight);
//
//            bot.rr.followTrajectory(alliance1ApproachJunction);
//            bot.slides.goDown();
//            sleep(100);
//            bot.claw.open();            //Cone placed
//            bot.rr.followTrajectory(alliance1GoBack);
//            bot.slides.runTo(580);
//            bot.rr.followTrajectory(alliance1GoToCone);
//            bot.rr.turn(Math.toRadians(90));
//            bot.rr.followTrajectory(alliance1GoForwardToCone);
//            bot.claw.close();
//            //new cone picked up
//            sleep(400);
//            bot.slides.runToTop();
//            bot.rr.turn(-Math.toRadians(90));
//            bot.rr.followTrajectory(alliance1GoToJunction);
//
//            bot.rr.followTrajectory(alliance1ApproachJunction);
//            bot.slides.goDown();
//            sleep(100);
//            bot.claw.open();
//            //cone placed
//            for(int i =0; i< 5; i++){
//                int runTo = 460;
//                bot.rr.followTrajectory(alliance1GoBack);
//                bot.slides.runTo(runTo);
//                bot.rr.followTrajectory(alliance1GoToCone);
//                bot.rr.turn(Math.toRadians(90));
//                bot.rr.followTrajectory(alliance1GoForwardToCone);
//                bot.claw.close();
//                //new cone picked up
//                sleep(400);
//                bot.slides.runToTop();
//                bot.rr.turn(-Math.toRadians(90));
//                bot.rr.followTrajectory(alliance1GoToJunction);
//
//                bot.rr.followTrajectory(alliance1ApproachJunction);
//                bot.slides.goDown();
//                sleep(100);
//                bot.claw.open();
//
//                runTo -= 70;
//            }
//            bot.rr.followTrajectory(alliance1GoBack);
//        }
//
//        else {
//
//            bot.slides.runToTop();
//            bot.rr.followTrajectory(alliance2StrafeLeft);
//
//            bot.rr.followTrajectory(alliance2ApproachJunction);
//            bot.slides.goDown();
//            sleep(100);
//            bot.claw.open();
//            //Cone placed
//            bot.rr.followTrajectory(alliance2GoBack);
//            bot.slides.runTo(580);
//            bot.rr.followTrajectory(alliance2GoToCone);
//            bot.rr.turn(Math.toRadians(90));
//            bot.rr.followTrajectory(alliance2GoForwardToCone);
//            bot.claw.close();
//            //new cone picked up
//            sleep(400);
//            bot.slides.runToTop();
//            bot.rr.turn(-Math.toRadians(90));
//            bot.rr.followTrajectory(alliance2GoToJunction);
//
//            bot.rr.followTrajectory(alliance2ApproachJunction);
//            bot.slides.goDown();
//            sleep(100);
//            bot.claw.open();
//            //cone placed
//            bot.rr.followTrajectory(alliance2GoBack);
//            bot.slides.runTo(460);
//            bot.rr.followTrajectory(alliance2GoToCone);
//            bot.rr.turn(Math.toRadians(90));
//            bot.rr.followTrajectory(alliance2GoForwardToCone);
//            bot.claw.close();
//            //new cone picked up
//            sleep(400);
//            bot.slides.runToTop();
//            bot.rr.turn(-Math.toRadians(90));
//            bot.rr.followTrajectory(alliance2GoToJunction);
//
//            bot.rr.followTrajectory(alliance2ApproachJunction);
//            bot.slides.goDown();
//            sleep(100);
//            bot.claw.open();
//            bot.rr.followTrajectory(alliance2GoBack);
//
//        }
//        bot.slides.runToBottom();
//        slidePeriodic.interrupt();
//        Thread runBack, outtake;
//        while(!isStopRequested() && opModeIsActive()){
//            gp2.readButtons();
//            if(gp2.wasJustPressed(GamepadKeys.Button.Y)){
//                bot.turret.runToAutoIntake();
//            }
//            if(gp2.wasJustPressed(GamepadKeys.Button.B)){
//                bot.claw.open();
//                bot.arm.intake();
//                bot.horizSlides.runToAutoIntake();
//            }
//            if(gp2.wasJustPressed(GamepadKeys.Button.A)){
//                bot.claw.close();
//                runBack = new Thread(() -> {try{
//                    Thread.sleep(1000);
//                }catch(InterruptedException ignored){
//
//                }
//                    bot.arm.storage(); bot.horizSlides.runToFullIn();});
//                runBack.start();
//            }
//            if(gp2.wasJustPressed(GamepadKeys.Button.X)){
//                bot.turret.runToAutoOuttake();
//                outtake = new Thread(() -> {try{
//                    Thread.sleep(1000);
//                }catch(InterruptedException e){}
//                    bot.arm.outtake();
//                    try{
//                        Thread.sleep(1000);
//                    }catch(InterruptedException e){}
//                    bot.arm.secure();
//                    bot.claw.open();});
//                outtake.start();
//
//            }
//        }

        bot.claw.close();

        Thread periodic = new Thread(() -> {
            while (opModeIsActive() && !isStopRequested()) {
                bot.slides.periodic();
                bot.turret.periodic();
                bot.horizSlides.periodic();
            }
        });

        periodic.start();

//        Pose2d startPose = new Pose2d(0, 0, 0);
//        drive.setPoseEstimate(startPose);
//        Trajectory forward = drive.trajectoryBuilder(startPose)
//                .forward(58)
//                .build();
//        drive.followTrajectory(forward);

//        sleep(500);

//        bot.turret.runTo(-650);
//        bot.slides.runToTop();
//        sleep(1000);
//        bot.arm.outtake();
//        sleep(200);
//        bot.claw.open();
//        telemetry.addLine("Scored preloaded");
////        sleep(1000);
//        bot.arm.storage();
//        bot.slides.runToBottom();
        outtake();
        for(int i = 4; i >= 0; i--){
            telemetry.addData("running cycle", i);
            telemetry.update();
            pickUpCone(i);
            outtake();
        }


    }
    private void outtake(){
        bot.turret.runToTurretAuto();
        bot.slides.runToTop();
        sleep(timeSlidesUp);
        bot.arm.outtake();
        sleep(timeOuttake);
        bot.arm.secure();
        bot.claw.open();
        sleep(timeConeDrop);
        bot.claw.close();
        bot.arm.storage();
        sleep(timeOuttake);
        bot.slides.runToBottom();
        bot.turret.runToAutoIntake();
        sleep(timeSlidesDown);
    }

    private void pickUpCone(int i){
        bot.claw.open();
        bot.arm.intakeAuto(i);
        sleep(timeIntakeDown);
        bot.horizSlides.runToAutoIntake();
        sleep(timeIntakeOut);
        bot.claw.close();
        sleep(timeIntakeClose);
        bot.arm.storage();
        sleep(timeIntakeUp);
        bot.horizSlides.runToFullIn();
        sleep(timeIntakeIn);
    }


//    void tagToTelemetry(AprilTagDetection detection)
//    {
//        telemetry.addLine(String.format("\nDetected tag ID=%d", detection.id));
//        telemetry.addLine(String.format("Translation X: %.2f feet", detection.pose.x*FEET_PER_METER));
//        telemetry.addLine(String.format("Translation Y: %.2f feet", detection.pose.y*FEET_PER_METER));
//        telemetry.addLine(String.format("Translation Z: %.2f feet", detection.pose.z*FEET_PER_METER));
//        telemetry.addLine(String.format("Rotation Yaw: %.2f degrees", Math.toDegrees(detection.pose.yaw)));
//        telemetry.addLine(String.format("Rotation Pitch: %.2f degrees", Math.toDegrees(detection.pose.pitch)));
//        telemetry.addLine(String.format("Rotation Roll: %.2f degrees", Math.toDegrees(detection.pose.roll)));
//    }

    }

