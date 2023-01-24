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
    boolean isRight;

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
            if(gp1.wasJustPressed(GamepadKeys.Button.X)){
                isRight=!isRight;
            }
            telemetry.addData("Current FPS:", camera.getFps());
            telemetry.addData("Current Max FPS:", camera.getCurrentPipelineMaxFps());
            telemetry.addData("Is right side?", isRight);

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
        bot.claw.close();



        Pose2d startPose = new Pose2d(0,0,0);
        Pose2d park1Pose = new Pose2d(12, 0,0);
        Pose2d park2Pose = new Pose2d(12, 12, 0);


        Trajectory forward = bot.rr.trajectoryBuilder(startPose)
                .forward(48)
                .build();


        //Alliance 1 trajectories
        Trajectory alliance1StrafeRight = bot.rr.trajectoryBuilder(forward.end())
                .strafeRight(13)
                .build();
        Trajectory alliance1ApproachJunction = bot.rr.trajectoryBuilder(alliance1StrafeRight.end())
                .forward(6)
                .build();
        Trajectory alliance1GoBack = bot.rr.trajectoryBuilder(alliance1ApproachJunction.end())
                .back(4)
                .build();
        Trajectory alliance1GoToCone = bot.rr.trajectoryBuilder(alliance1GoBack.end())
                .strafeLeft(28)
                .build();
        Trajectory alliance1GoForwardToCone = bot.rr.trajectoryBuilder(new Pose2d(alliance1GoToCone.end().getX(), alliance1GoToCone.end().getY(), Math.toRadians(90)))
                .forward(4)
                .build();
        Trajectory alliance1GoToJunction = bot.rr.trajectoryBuilder(new Pose2d(alliance1GoForwardToCone.end().getX(), alliance1GoForwardToCone.end().getY(), Math.toRadians(0)))
                .strafeRight(32)
                .build();

       /* Trajectory goBack = bot.rr.trajectoryBuilder(new Pose2d(allianceOneGoToJunction.end().getX(), allianceOneGoToJunction.end().getY(), -Math.toRadians(90)))
                .back(6)
                .build();
        */

        //Alliance 2 trajectories
        Trajectory alliance2StrafeLeft= bot.rr.trajectoryBuilder(forward.end())
                .strafeLeft(13)
                .build();
        Trajectory alliance2ApproachJunction= bot.rr.trajectoryBuilder(alliance2StrafeLeft.end())
                .forward(6)
                .build();
        Trajectory alliance2GoBack = bot.rr.trajectoryBuilder(alliance2ApproachJunction.end())
                .back(4)
                .build();
        Trajectory alliance2GoToCone = bot.rr.trajectoryBuilder(alliance2GoBack.end())
                .strafeRight(28)
                .build();
        Trajectory alliance2GoForwardToCone = bot.rr.trajectoryBuilder(new Pose2d(alliance2GoToCone.end().getX(), alliance2GoToCone.end().getY(), -Math.toRadians(90)))
                .forward(4)
                .build();
        Trajectory alliance2GoToJunction = bot.rr.trajectoryBuilder(new Pose2d(alliance2GoForwardToCone.end().getX(), alliance2GoForwardToCone.end().getY(), Math.toRadians(0)))
                .strafeLeft(32)
                .build();


        bot.rr.followTrajectory(forward);

        if(!isRight) {
            bot.slide.runToTop();
            bot.rr.followTrajectory(alliance1StrafeRight);

            bot.rr.followTrajectory(alliance1ApproachJunction);
            bot.slide.goDown();
            sleep(100);
            bot.claw.open();
            //Cone placed
            bot.rr.followTrajectory(alliance1GoBack);
            bot.slide.runTo(580);
            bot.rr.followTrajectory(alliance1GoToCone);
            bot.rr.turn(Math.toRadians(90));
            bot.rr.followTrajectory(alliance1GoForwardToCone);
            bot.claw.close();
            //new cone picked up
            sleep(400);
            bot.slide.runToTop();
            bot.rr.turn(-Math.toRadians(90));
            bot.rr.followTrajectory(alliance1GoToJunction);

            bot.rr.followTrajectory(alliance1ApproachJunction);
            bot.slide.goDown();
            sleep(100);
            bot.claw.open();
            //cone placed
          for(int i =0; i< 5; i++){
              int runTo = 460;
              bot.rr.followTrajectory(alliance1GoBack);
              bot.slide.runTo(runTo);
              bot.rr.followTrajectory(alliance1GoToCone);
              bot.rr.turn(Math.toRadians(90));
              bot.rr.followTrajectory(alliance1GoForwardToCone);
              bot.claw.close();
              //new cone picked up
              sleep(400);
              bot.slide.runToTop();
              bot.rr.turn(-Math.toRadians(90));
              bot.rr.followTrajectory(alliance1GoToJunction);

              bot.rr.followTrajectory(alliance1ApproachJunction);
              bot.slide.goDown();
              sleep(100);
              bot.claw.open();

              runTo -= 100;
          }
            bot.rr.followTrajectory(alliance1GoBack);
        }

        else {

            bot.slide.runToTop();
            bot.rr.followTrajectory(alliance2StrafeLeft);

            bot.rr.followTrajectory(alliance2ApproachJunction);
            bot.slide.goDown();
            sleep(100);
            bot.claw.open();
            //Cone placed
            bot.rr.followTrajectory(alliance2GoBack);
            bot.slide.runTo(580);
            bot.rr.followTrajectory(alliance2GoToCone);
            bot.rr.turn(Math.toRadians(90));
            bot.rr.followTrajectory(alliance2GoForwardToCone);
            bot.claw.close();
            //new cone picked up
            sleep(400);
            bot.slide.runToTop();
            bot.rr.turn(-Math.toRadians(90));
            bot.rr.followTrajectory(alliance2GoToJunction);

            bot.rr.followTrajectory(alliance2ApproachJunction);
            bot.slide.goDown();
            sleep(100);
            bot.claw.open();
            //cone placed
            bot.rr.followTrajectory(alliance2GoBack);
            bot.slide.runTo(460);
            bot.rr.followTrajectory(alliance2GoToCone);
            bot.rr.turn(Math.toRadians(90));
            bot.rr.followTrajectory(alliance2GoForwardToCone);
            bot.claw.close();
            //new cone picked up
            sleep(400);
            bot.slide.runToTop();
            bot.rr.turn(-Math.toRadians(90));
            bot.rr.followTrajectory(alliance2GoToJunction);

            bot.rr.followTrajectory(alliance2ApproachJunction);
            bot.slide.goDown();
            sleep(100);
            bot.claw.open();
            bot.rr.followTrajectory(alliance2GoBack);

        }
        bot.slide.runToBottom();
        slidePeriodic.interrupt();
    }

}

