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
import org.firstinspires.ftc.teamcode.teleop.subsystems.Turret;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;

//@Config
//@Autonomous(name="Jaiden's Auto")
public class AlternateAuto extends LinearOpMode {

    Bot bot;
    boolean isRight;
    SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

    public static int timeSlidesUp = 800, timeSlidesDown = 550, timeOuttake = 350, timeConeDrop = 250, timeIntakeDown = 200, timeIntakeOut = 700, timeIntakeClose = 250, timeIntakeUp = 300, timeIntakeIn = 400;

//
//    static final double FEET_PER_METER = 3.28084;
//
//    double fx = 1078.03779;
//    double fy = 1084.50988;
//    double cx = 580.850545;
//    double cy = 245.959325;
//
//    // UNITS ARE METERS
//    double tagsize = 0.032; //ONLY FOR TESTING
//
//    // Tag ID 1,2,3 from the 36h11 family
//    int ID_ONE = 1;
//    int ID_TWO = 2;
//    int ID_THREE = 3;
//    AprilTagDetection tagOfInterest = null;

    @Override
    public void runOpMode() throws InterruptedException {
//        telemetry.setAutoClear(true);
//        bot = Bot.getInstance(this);
//        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
//
//        // Retrieve the IMU from the hardware map
////        BNO055IMU imu = hardwareMap.get(BNO055IMU.class, "imu");
////        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
////        // Technically this is the default, however specifying it is clearer
////        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
////        // Without this, data retrieving from the IMU throws an exception
////        imu.initialize(parameters);
//
//        GamepadEx gp1 = new GamepadEx(gamepad1);
//        GamepadEx gp2 = new GamepadEx(gamepad2);
//        bot.arm.preload();
//        while(!isStarted()){
//            gp1.readButtons();
//            if(gp1.wasJustPressed(GamepadKeys.Button.A)){
//                bot.claw.close();
//            }
//        }
//
//        waitForStart();
//
//
//        bot.claw.close();
//
//        Thread periodic = new Thread(() -> {
//            while (opModeIsActive() && !isStopRequested()) {
//                bot.slides.periodic();
//                bot.turret.periodic();
//                bot.horizSlides.periodic();
//            }
//        });
//
//        periodic.start();
//
//        drive.followTrajectory(trajectory);
//
////        outtake();
////        for(int i = 4; i >= 0; i--){
////            telemetry.addData("running cycle", i);
////            telemetry.update();
////            pickUpCone(i);
////            outtake();
////        }
//
//
//    }
//    private void outtake(){
//        bot.turret.runToTurretAuto();
//        bot.slides.runToTop();
//        sleep(timeSlidesUp);
//        bot.arm.outtake();
//        sleep(timeOuttake);
//        bot.arm.secure();
//        bot.claw.open();
//        sleep(timeConeDrop);
//        bot.claw.close();
//        bot.arm.storage();
//        sleep(timeOuttake);
//        bot.slides.runToBottom();
//        bot.turret.runToAutoIntake();
//        sleep(timeSlidesDown);
//    }
//
//    private void pickUpCone(int i){
//        bot.claw.open();
//        bot.arm.intakeAuto(i);
//        sleep(timeIntakeDown);
//        bot.horizSlides.runToAutoIntake();
//        sleep(timeIntakeOut);
//        bot.claw.close();
//        sleep(timeIntakeClose);
//        bot.arm.storage();
//        sleep(timeIntakeUp);
//        bot.horizSlides.runToFullIn();
//        sleep(timeIntakeIn);
    }
//
//    private void step1() {
//        bot.slides.runTo(-338);
//        bot.turret.runToAutoIntake();
//        bot.horizSlides.runToAutoIntake();
//    }
//
//    private void step2() {
//        bot.arm.lift(0.61);
//        bot.turret.runToAutoOuttake();
//        bot.horizSlides.runToFullIn();
//        bot.slides.runToTop();
//    }
//
//
//    private void pickUpCone2(int i) {
//        bot.arm.intakeAuto(i);
//        sleep(timeIntakeIn);
//        bot.claw.close();
//    }
//
//    Trajectory trajectory = drive.trajectoryBuilder(new Pose2d(0,0,0), 0)
//            .lineTo(
//                    new Vector2d(50, 2),
//                    SampleMecanumDrive.getVelocityConstraint(15, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
//                    SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
//            )
//            .addDisplacementMarker(30, () -> {
//                bot.turret.runToTurretAuto();
//            })
//            .addDisplacementMarker(48, () -> {
//                bot.slides.runToTop();
//                sleep(timeSlidesUp);
//                bot.arm.outtake();
//                sleep(timeOuttake);
//                bot.claw.open();
//                for(int i = 4; i >= 0; i--){
//                    telemetry.addData("running cycle", i);
//                    telemetry.update();
//                    step1();
//                    pickUpCone2(i);
//                    step2();
//                    bot.arm.outtake();
//                    sleep(timeOuttake);
//                    bot.claw.open();
//                }
//            })
//            .build();

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


