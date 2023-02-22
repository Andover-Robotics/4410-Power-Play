package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.teleop.subsystems.Bot;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;


@Config
@Autonomous(name="MainAutonomous")
public class MainAutonomous extends LinearOpMode {

    Bot bot;
    double turretPos;

    enum Side{
        RIGHT, LEFT, NULL;
    }

    Side side = Side.NULL;

    public static int driveTime = 2000, timeSlidesUp = 800, timeSlidesDown = 550, timeOuttake = 325, timeConeDrop = 150, timeIntakeDown = 200, timeIntakeOut = 700, timeIntakeClose = 150, timeIntakeUp = 500, timeIntakeIn = 400;


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

        //CAMERA STUFF =====================

        WebcamName camName = hardwareMap.get(WebcamName.class, "Webcam 1");
        OpenCvCamera camera = OpenCvCameraFactory.getInstance().createWebcam(camName);
        AprilTagDetectionPipeline aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);

        camera.setPipeline(aprilTagDetectionPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                camera.startStreaming(1280,720, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode)
            {

            }
        });

//        telemetry.setMsTransmissionInterval(50);


        while(!isStarted()) {
            gp1.readButtons();
            if(gp1.wasJustPressed(GamepadKeys.Button.B)){
                side = Side.RIGHT;
            }
            if(gp1.wasJustPressed(GamepadKeys.Button.X)){
                side = Side.LEFT;
            }

            if(gp1.wasJustPressed(GamepadKeys.Button.A)){
                bot.claw.close();
            }
            //TODO add just park

            telemetry.addData("Current FPS:", camera.getFps());
            telemetry.addData("Current Max FPS:", camera.getCurrentPipelineMaxFps());
            telemetry.addData("side?", side.toString());

            ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();

            if(currentDetections.size() != 0)
            {
                boolean tagFound = false;

                for(AprilTagDetection tag : currentDetections)
                {
                    if(tag.id == ID_ONE || tag.id == ID_TWO || tag.id == ID_THREE)
                    {
                        tagOfInterest = tag;
                        tagFound = true;
                        break;
                    }
                }

                if(tagFound)
                {
                    telemetry.addLine("Tag of interest is in sight!\n\nLocation data:");
                    tagToTelemetry(tagOfInterest);
                }
                else
                {
                    telemetry.addLine("Don't see tag of interest :(");

                    if(tagOfInterest == null)
                    {
                        telemetry.addLine("(The tag has never been seen)");
                    }
                    else
                    {
                        telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                        tagToTelemetry(tagOfInterest);
                    }
                }

            }
            else
            {
                telemetry.addLine("Don't see tag of interest :(");

                if(tagOfInterest == null)
                {
                    telemetry.addLine("(The tag has never been seen)");
                }
                else
                {
                    telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                    tagToTelemetry(tagOfInterest);
                }

            }

            telemetry.update();
            sleep(20);

        }

        camera.stopStreaming();
        camera.closeCameraDevice();

        //END CAMERA STUFF ===============

        waitForStart();
        if(!isStopRequested()) {


            bot.claw.close();

            Thread periodic = new Thread(() -> {
                while (opModeIsActive() && !isStopRequested()) {
                    bot.slides.periodic();
                    bot.turret.periodic();
                    bot.horizSlides.periodic();
                    turretPos = bot.turret.getPosition();
                }
            });

            periodic.start();

            Pose2d startPose = new Pose2d(0, 0, 0);
            drive.setPoseEstimate(startPose);
            Trajectory forward = drive.trajectoryBuilder(startPose)
//                    .forward(52)
                    .lineTo(
                            new Vector2d(52, 3),
                            SampleMecanumDrive.getVelocityConstraint(20, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                            SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                    )
                    .addDisplacementMarker(36, () -> {
                        bot.slides.runToTop();
                    })
                    .addDisplacementMarker(() -> {
                        bot.arm.outtake();
                        bot.claw.open();
                        bot.arm.secure();
                    })
                    .build();

            Trajectory parkLeft = drive.trajectoryBuilder(forward.end())
                    .strafeLeft(24)
                    .build();
            Trajectory parkRight = drive.trajectoryBuilder(forward.end())
                    .strafeRight(24)
                    .build();

            Thread driveForward = new Thread(() -> drive.followTrajectory(forward));
            driveForward.start();


            if (side != Side.NULL) {
//                outtake(5);
                for (int i = 4; i >= 0; i--) {
                    telemetry.addData("running cycle", i);
                    telemetry.update();
                    pickUpCone(i);
                    outtake(i);
                }
            } else {
                sleep(3000);
            }

            if (tagOfInterest.id == ID_ONE) {
                drive.followTrajectory(parkLeft);
            } else if (tagOfInterest.id == ID_THREE) {
                drive.followTrajectory(parkRight);
            } else {
                sleep(1500);
            }
        }
    }
    private void outtake(int i){
        if(side==Side.RIGHT) {
            bot.turret.runToAutoOuttakeRight();
        }else{
            bot.turret.runToAutoOuttakeLeft();
        }
//        bot.slides.runToTop();
        sleep(timeSlidesUp);
        if(i == 5){
            sleep(200);
        }
        bot.arm.outtake();
        sleep(timeOuttake);
        bot.arm.secure();
        bot.claw.open();
        bot.arm.storage();
        sleep(timeConeDrop);
        bot.claw.close();
        sleep(timeOuttake);
        bot.slides.runToLow();
        if(side==Side.RIGHT) {
            bot.turret.runToAutoIntakeRight();
            while(turretPos != 830)
            {
                if(turretPos == 205){
                    bot.horizSlides.runToAutoIntake();
                    sleep(timeIntakeOut);
                }
            }
        }else {
            bot.turret.runToAutoIntakeLeft();
            while (turretPos != -830) {
                if (turretPos == -205) {
                    bot.horizSlides.runToAutoIntake();
                    sleep(timeIntakeOut);
                }
            }
        }
        sleep(timeSlidesDown);
    }

    private void pickUpCone(int i){
        bot.claw.open();
        bot.arm.intakeAuto(i);
        sleep(timeIntakeDown);
//        bot.horizSlides.runToAutoIntake();
//        sleep(timeIntakeOut);
        bot.claw.close();
        sleep(timeIntakeClose);
        bot.slides.runToTop();
        bot.arm.autoStorage();
        if(i > 0) {
            sleep(timeIntakeUp);
        }
        bot.horizSlides.runToFullIn();
        sleep(timeIntakeIn);
    }


    void tagToTelemetry(AprilTagDetection detection)
    {
        telemetry.addLine(String.format("\nDetected tag ID=%d", detection.id));
        telemetry.addLine(String.format("Translation X: %.2f feet", detection.pose.x*FEET_PER_METER));
        telemetry.addLine(String.format("Translation Y: %.2f feet", detection.pose.y*FEET_PER_METER));
        telemetry.addLine(String.format("Translation Z: %.2f feet", detection.pose.z*FEET_PER_METER));
        telemetry.addLine(String.format("Rotation Yaw: %.2f degrees", Math.toDegrees(detection.pose.yaw)));
        telemetry.addLine(String.format("Rotation Pitch: %.2f degrees", Math.toDegrees(detection.pose.pitch)));
        telemetry.addLine(String.format("Rotation Roll: %.2f degrees", Math.toDegrees(detection.pose.roll)));
    }

    }

