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
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;

import java.util.Map;

@Autonomous(name="MainAutonomous")
public class MainAutonomous extends LinearOpMode {

    Bot bot;
    boolean isAlliance1;
    GamepadEx gamepad= new GamepadEx(gamepad1);
    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.setAutoClear(true);

        bot = Bot.getInstance(this);

        // Retrieve the IMU from the hardware map
//        BNO055IMU imu = hardwareMap.get(BNO055IMU.class, "imu");
//        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
//        // Technically this is the default, however specifying it is clearer
//        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
//        // Without this, data retrieving from the IMU throws an exception
//        imu.initialize(parameters);

        GamepadEx gp1 = new GamepadEx(gamepad1);

        //CAMERA STUFF =====================
        TestPipeline pipeline = new TestPipeline(telemetry);

        WebcamName camName = hardwareMap.get(WebcamName.class, "Webcam 1");
        OpenCvCamera camera = OpenCvCameraFactory.getInstance().createWebcam(camName);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.setPipeline(pipeline);
                camera.startStreaming(320 * 3, 240 * 3, OpenCvCameraRotation.UPRIGHT);
                telemetry.addData("Camera Status", "Opened");
            }

            @Override
            public void onError(int errorCode) {
                telemetry.addData("Error Code", errorCode);
            }
        });



        while(!isStarted()) {
            if(gamepad.wasJustPressed(GamepadKeys.Button.X)){
                isAlliance1=true;
            }
            telemetry.addData("Current FPS:", camera.getFps());
            telemetry.addData("Current Max FPS:", camera.getCurrentPipelineMaxFps());

            if (pipeline.getSignalVal() == TestPipeline.SignalVal.GREEN) {
                telemetry.addData("The signal is green with a percentage of", TestPipeline.greenPercentage);
            }
            else if(pipeline.getSignalVal() == TestPipeline.SignalVal.PINK){
                telemetry.addData("The signal is pink with a percentage of", TestPipeline.pinkPercentage);
            }
            else if(pipeline.getSignalVal() == TestPipeline.SignalVal.YELLOW){
                telemetry.addData("The signal is yellow with a percentage of", TestPipeline.yellowPercentage);
            }

            gp1.readButtons();
            if(gp1.wasJustPressed(GamepadKeys.Button.Y)) {
                bot.claw.close();
            }

            telemetry.update();

        }

        camera.stopStreaming();
        camera.closeCameraDevice();

        //END CAMERA STUFF ===============

        waitForStart();

        bot.claw.close();

        Thread slidePeriodic = new Thread(() -> {
            while(opModeIsActive()){
                bot.slide.periodic();
            }
        });
        slidePeriodic.start();



        Pose2d startPose = new Pose2d(0,0,0);
        Pose2d park1Pose = new Pose2d(12, 0,0);
        Pose2d park2Pose = new Pose2d(12, 12, 0);



        Trajectory forward = bot.rr.trajectoryBuilder(startPose)
                .forward(48)
                .build();
        Trajectory alliance1StrafeRight = bot.rr.trajectoryBuilder(new Pose2d(48,0,0))
                .strafeRight(12)
                .build();
        Trajectory alliance1ApproachJunction = bot.rr.trajectoryBuilder(new Pose2d(48, -12, 0))
                .forward(4)
                .build();
        Trajectory alliance1GoBack = bot.rr.trajectoryBuilder(new Pose2d(52, -12, 0))
                .back(4)
                .build();
        Trajectory alliance2StrafeLeft= bot.rr.trajectoryBuilder(new Pose2d(48,0,0))
                .strafeLeft(12)
                .build();
        Trajectory alliance2ApproachJunction= bot.rr.trajectoryBuilder(new Pose2d(48, 12, 0))
                .forward(4)
                .build();
        Trajectory alliance2GoBack = bot.rr.trajectoryBuilder(new Pose2d(52, 12, 0))
                .back(4)
                .build();
        Trajectory goToCone = bot.rr.trajectoryBuilder(new Pose2d(44,-12,0))
                .strafeLeft(15)
                .build();
        Trajectory goToJunction = bot.rr.trajectoryBuilder(new Pose2d(49,-12,0))
                .strafeRight(15)
                .build();

        bot.rr.followTrajectory(forward);

        if(isAlliance1) {
            bot.rr.followTrajectory(alliance1StrafeRight);
            bot.slide.runToTop();

            sleep(3000);

            bot.rr.followTrajectory(alliance1ApproachJunction);
            telemetry.addLine("Slides going up");
            telemetry.update();
            bot.slide.goDown();

            sleep(1000);

            telemetry.addLine("Slides going down");
            telemetry.update();
            bot.claw.open();

            sleep(1000);
            bot.rr.followTrajectory(alliance1GoBack);
            telemetry.addLine("Slides going to bottom");
            telemetry.update();
            bot.slide.runToLow();
            sleep(3000);

            bot.rr.followTrajectory(goToCone);
            bot.slide.runTo(580);
            sleep(1000);
            bot.claw.close();
            bot.slide.runToTop();
            bot.rr.followTrajectory(goToJunction);
            bot.rr.followTrajectory(alliance1ApproachJunction);
            bot.slide.goDown();
            bot.claw.open();
        }

        else {

            bot.rr.followTrajectory(alliance2StrafeLeft);
            bot.slide.runToTop();

            sleep(3000);

            bot.rr.followTrajectory(alliance2ApproachJunction);
            telemetry.addLine("Slides going up");
            telemetry.update();
            bot.slide.goDown();

            sleep(1000);

            telemetry.addLine("Slides going down");
            telemetry.update();
            bot.claw.open();

            sleep(1000);
            bot.rr.followTrajectory(alliance2GoBack);
            telemetry.addLine("Slides going to bottom");
            telemetry.update();
            bot.slide.runToLow();
            sleep(3000);
            slidePeriodic.interrupt();
        }
    }

}

