package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.auto.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.auto.trajectorysequence.TrajectorySequenceBuilder;
import org.firstinspires.ftc.teamcode.teleop.subsystems.Bot;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;

import java.util.ArrayList;
import java.util.Map;

@Autonomous(name="MainAutonomous")
public class MainAutonomous extends LinearOpMode {

    Bot bot;
    boolean isAllianceOne;

    static final double FEET_PER_METER = 3.28084;

    double fx = 1078.03779;
    double fy = 1084.50988;
    double cx = 580.850545;
    double cy = 245.959325;
    //
//    // UNITS ARE METERS
    double tagsize = 0.032; //ONLY FOR TESTING
    //
//    // Tag ID 1,2,3 from the 36h11 family
    int ID_ONE = 1;
    int ID_TWO = 2;
    int ID_THREE = 3;
    AprilTagDetection tagOfInterest = null;


    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.setAutoClear(true);
        bot = Bot.getInstance(this);

        // Retrieve the IMU from the hardware map
        BNO055IMU imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
//        // Technically this is the default, however specifying it is clearer
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
//        // Without this, data retrieving from the IMU throws an exception
        imu.initialize(parameters);

        GamepadEx gp1 = new GamepadEx(gamepad1);

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


        telemetry.setMsTransmissionInterval(50);


        while (!isStarted()) {
            if (gp1.wasJustPressed(GamepadKeys.Button.X)) {
                isAllianceOne = !isAllianceOne;
            }
            telemetry.addData("Current FPS:", camera.getFps());
            telemetry.addData("Current Max FPS:", camera.getCurrentPipelineMaxFps());
            telemetry.addData("Is right side?", isAllianceOne);

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
                } else {
                    telemetry.addLine("Don't see tag of interest :(");

                    if (tagOfInterest == null) {
                        telemetry.addLine("(The tag has never been seen)");
                    } else {
                        telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                    }
                }

            } else {
                telemetry.addLine("Don't see tag of interest :(");

                if (tagOfInterest == null) {
                    telemetry.addLine("(The tag has never been seen)");
                } else {
                    telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                }

            }


            gp1.readButtons();
            if (gp1.wasJustPressed(GamepadKeys.Button.Y)) {
                bot.claw.close();
            }

            telemetry.update();

        }

        camera.stopStreaming();
        camera.closeCameraDevice();

        //END CAMERA STUFF ===============

        waitForStart();

        bot.claw.close();
//
//        Thread slidePeriodic = new Thread(() -> {
//            while (opModeIsActive()) {
//                bot.slide.periodic();
//            }
//        });
//        slidePeriodic.start();
        bot.claw.close();


        Pose2d startPose = new Pose2d(0, 0, 0);
        Pose2d park1Pose = new Pose2d(12, 0, 0);
        Pose2d park2Pose = new Pose2d(12, 12, 0);


        Trajectory forward = bot.rr.trajectoryBuilder(startPose)
                .forward(25)
                .build();


        //Alliance 1 trajectories
        Trajectory alliance1StrafeRight = bot.rr.trajectoryBuilder(forward.end())
                .strafeRight(12)
                .build();
        Trajectory alliance1ApproachJunction = bot.rr.trajectoryBuilder(alliance1StrafeRight.end())
                .forward(6)
                .build();
        Trajectory alliance1GoBack = bot.rr.trajectoryBuilder(alliance1ApproachJunction.end())
                .back(4.75)
                .build();
        Trajectory alliance1GoToCone = bot.rr.trajectoryBuilder(new Pose2d(alliance1GoBack.end().getX(), alliance1GoBack.end().getY(), Math.toRadians(90)))
                .forward(38.55)
                .build();
        Trajectory alliance1GoToJunction = bot.rr.trajectoryBuilder(new Pose2d(alliance1GoToCone.end().getX(), alliance1GoToCone.end().getY(), Math.toRadians(90)))
                .back(34.5)
                .build();

       /* Trajectory goBack = bot.rr.trajectoryBuilder(new Pose2d(allianceOneGoToJunction.end().getX(), allianceOneGoToJunction.end().getY(), -Math.toRadians(90)))
                .back(6)
                .build();
        */

        //Alliance 2 trajectories
        Trajectory alliance2StrafeLeft = bot.rr.trajectoryBuilder(forward.end())
                .strafeLeft(12)
                .build();
        Trajectory alliance2ApproachJunction = bot.rr.trajectoryBuilder(alliance2StrafeLeft.end())
                .forward(6)
                .build();
        Trajectory alliance2GoBack = bot.rr.trajectoryBuilder(alliance2ApproachJunction.end())
                .back(4.75)
                .build();
        Trajectory alliance2GoToCone = bot.rr.trajectoryBuilder(new Pose2d(alliance2GoBack.end().getX(), alliance2GoBack.end().getY(), -Math.toRadians(90)))
                .forward(38.55)
                .build();
       /* Trajectory alliance2GoForwardToCone = bot.rr.trajectoryBuilder(new Pose2d(alliance2GoToCone.end().getX(), alliance2GoToCone.end().getY(), -Math.toRadians(90)))
                .forward(12)
                .build();
        */
        Trajectory alliance2GoToJunction = bot.rr.trajectoryBuilder(new Pose2d(alliance2GoToCone.end().getX(), alliance2GoToCone.end().getY(), -Math.toRadians(90)))
                .back(34.5)
                .build();

        Trajectory readyForCVAllianceLeft1 = bot.rr.trajectoryBuilder(new Pose2d(alliance1GoBack.end().getX(), alliance1GoBack.end().getY(), Math.toRadians(0)))
                .strafeRight(12)
                .build();
        Trajectory readyForCVAllianceOne = bot.rr.trajectoryBuilder(new Pose2d(readyForCVAllianceLeft1.end().getX(), readyForCVAllianceLeft1.end().getY(), Math.toRadians(0)))
                .back(24)
                .build();
        Trajectory readyforCVAllianceRight2 = bot.rr.trajectoryBuilder(new Pose2d(alliance2GoBack.end().getX(), alliance2GoBack.end().getY(), Math.toRadians(0)))
                .strafeLeft(12)
                .build();
        Trajectory readyForCVAllianceTwo = bot.rr.trajectoryBuilder(new Pose2d(readyforCVAllianceRight2.end().getX(), readyforCVAllianceRight2.end().getY(), Math.toRadians(0)))
                .back(24)
                .build();


        Trajectory allianceOneId1 = bot.rr.trajectoryBuilder(new Pose2d(forward.end().getX(), forward.end().getY(), Math.toRadians(0)))
                .strafeLeft(24)
                .build();
        Trajectory allianceOneId2 = bot.rr.trajectoryBuilder(new Pose2d(forward.end().getX(), forward.end().getY(), Math.toRadians(0)))
                .strafeLeft(15)
                .build();

        Trajectory allianceOneId3 = bot.rr.trajectoryBuilder(new Pose2d(forward.end().getX(), forward.end().getY(), Math.toRadians(0)))
                .strafeRight(24)
                .build();

        Trajectory allianceTwoId1 = bot.rr.trajectoryBuilder(new Pose2d(forward.end().getX(), alliance2GoBack.end().getY(), Math.toRadians(0)))
                .strafeLeft(27)
                .build();
        Trajectory allianceTwoId2 = bot.rr.trajectoryBuilder(new Pose2d(alliance2GoBack.end().getX(), alliance2GoBack.end().getY(), Math.toRadians(0)))
                .strafeRight(13.5)
                .build();
        Trajectory allianceTwoId3 = bot.rr.trajectoryBuilder(new Pose2d(alliance2GoBack.end().getX(), alliance2GoBack.end().getY(), Math.toRadians(0)))
                .strafeRight(21)
                .build();

//    if (!isAllianceOne) {
        bot.rr.followTrajectory(forward);
//        bot.slide.runToTop();
//        bot.rr.followTrajectory(alliance1StrafeRight);
//        bot.rr.followTrajectory(alliance1ApproachJunction);
//        bot.slide.goDown();
//        bot.claw.open();
//        //Cone placed
//        bot.slide.runTo(495);
//        bot.rr.followTrajectory(alliance1GoBack);
//        bot.rr.turn(Math.toRadians(90));
//        bot.rr.followTrajectory(alliance1GoToCone);
//        bot.claw.close();
//        sleep(600);
//        //new cone picked up
//        bot.slide.runToTop();
//        bot.rr.followTrajectory(alliance1GoToJunction);
//        bot.rr.turn(-Math.toRadians(90));
//        bot.rr.followTrajectory(alliance1ApproachJunction);
//        bot.slide.goDown();
//        bot.claw.open();
//        bot.slide.runToBottom();
//    } else if(isAllianceOne) {
//        bot.rr.followTrajectory(forward);
//        bot.slide.runToTop();
//        bot.rr.followTrajectory(alliance2StrafeLeft);
//        bot.rr.followTrajectory(alliance2ApproachJunction);
//        bot.slide.goDown();
//        bot.claw.open();
//        //Cone placed
//        bot.slide.runTo(495);
//        bot.rr.followTrajectory(alliance2GoBack);
//        bot.rr.turn(-Math.toRadians(90));
//        bot.rr.followTrajectory(alliance2GoToCone);
//        bot.claw.close();
//        sleep(600);
//        //new cone picked up
//        bot.slide.runToTop();
//        bot.rr.followTrajectory(alliance2GoToJunction);
//        bot.rr.turn(Math.toRadians(90));
//        sleep(1500);
//        bot.rr.followTrajectory(alliance2ApproachJunction);
//        bot.slide.goDown();
//        bot.claw.open();
//        bot.rr.followTrajectory(alliance2GoBack);
//        sleep(500);
////        bot.slide.runToBottom();
//    }

//    slidePeriodic.interrupt();

//        if(!isAllianceOne){
//            bot.rr.followTrajectory(readyForCVAllianceOne);
//        }
//        else{
//            bot.rr.followTrajectory(readyForCVAllianceTwo);
//        }
////
//        if(tagOfInterest.id == ID_ONE){
//            if(!isAllianceOne){
//                bot.rr.followTrajectory(readyforCVAllianceRight2);
//                bot.rr.followTrajectory(readyForCVAllianceTwo);
//                bot.rr.followTrajectory(allianceTwoId1);
//            }
//            if(isAllianceOne){
//                bot.rr.followTrajectory(readyForCVAllianceLeft1);
//                bot.rr.followTrajectory(readyForCVAllianceOne);
//                bot.rr.followTrajectory(allianceOneId1);
//            }
//
//        }
//
//        else if (tagOfInterest.id == ID_THREE){
//            if(!isAllianceOne){
//                bot.rr.followTrajectory(readyforCVAllianceRight2);
//                bot.rr.followTrajectory(readyForCVAllianceTwo);
//                bot.rr.followTrajectory(allianceTwoId3);
//            }
//            if(isAllianceOne){
//                bot.rr.followTrajectory(readyForCVAllianceLeft1);
//                bot.rr.followTrajectory(readyForCVAllianceOne);
//                bot.rr.followTrajectory(allianceOneId3);
//            }
//
////        }
//    if (isAllianceOne) {
        if(tagOfInterest.id == ID_ONE){
            bot.rr.followTrajectory(allianceOneId1);
        }else if(tagOfInterest.id == ID_TWO){
//            bot.rr.followTrajectory(allianceOneId2);
        }else if(tagOfInterest.id == ID_THREE){
            bot.rr.followTrajectory(allianceOneId3);
        }
//        else{
////            bot.rr.followTrajectory(allianceOneId2);
//        }
//
//    } else if (!isAllianceOne) {
//        if(tagOfInterest.id == ID_ONE){
//            bot.rr.followTrajectory(allianceTwoId1);
//        }else if(tagOfInterest.id == ID_TWO){
//            bot.rr.followTrajectory(allianceTwoId2);
//        }else if(tagOfInterest.id == ID_THREE){
//            bot.rr.followTrajectory(allianceTwoId3);
//        }
//        else{
//            bot.rr.followTrajectory(allianceTwoId2);
//        }
//    }
}
//    }




}
