package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.teleop.subsystems.Bot;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;

import java.util.Map;

@Autonomous(name="MainAutonomousPark")
public class MainAutonomous extends LinearOpMode {

    Bot bot;
    boolean terminal = false;
    @Override
    public void runOpMode() throws InterruptedException {
        for (Map.Entry<String, DcMotor> entry : hardwareMap.dcMotor.entrySet()) {
            entry.getValue().setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            while (!isStopRequested() && Math.abs(entry.getValue().getCurrentPosition()) > 1) {
                idle();
            }
            telemetry.addData(entry.getKey(), "is reset");
            telemetry.update();
        }

        Bot.instance = null;
        bot = Bot.getInstance(this);
        Thread closeClaw = new Thread(bot.claw::close);
        closeClaw.start();

        GamepadEx gp1 = new GamepadEx(gamepad1);

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

            /*telemetry.addData("\nDo terminal (press X to switch)", terminal);
            gp1.readButtons();
            if(gp1.wasJustPressed(GamepadKeys.Button.X)){
                terminal = !terminal;
            }
            telemetry.update();
             */
        }
        Thread slidePeriodic = new Thread(() -> {
            while(opModeIsActive()){
                bot.slide.periodic();
            }
        });
        slidePeriodic.start();

        camera.stopStreaming();
        camera.closeCameraDevice();

        Pose2d startPose = new Pose2d();
        Trajectory moveOut = bot.rr.trajectoryBuilder(startPose)
                .strafeLeft(40).build(),
                score = bot.rr.trajectoryBuilder(new Pose2d(0, 40))//change this for more complex paths
                        .forward(4).build(),
                goBack = bot.rr.trajectoryBuilder(new Pose2d(4, 40))
                        .back(4).build(),
                strafeRight = bot.rr.trajectoryBuilder(new Pose2d(0, 40))
                        .strafeRight(12).build(),
                parkRight = bot.rr.trajectoryBuilder(new Pose2d(0, 28))
                        .forward(24).build(),
                parkLeft = bot.rr.trajectoryBuilder(new Pose2d(0, 28))
                        .back(24).build();

        Trajectory strafeLeftfirst = bot.rr.trajectoryBuilder(startPose)
                .strafeLeft(28).build();



            bot.rr.followTrajectory(moveOut);
            Thread runToTop = new Thread(bot.slide::runToTop);
            runToTop.start();
            telemetry.addData("Started slide",runToTop);
            telemetry.update();
            sleep(3000);
            bot.rr.followTrajectory(score);
            //bot.claw.open();
            Thread openClaw = new Thread(bot.claw::open);
            telemetry.addData("Started claw", openClaw);
            telemetry.update();
            openClaw.start();
            sleep(1000);
            bot.rr.followTrajectory(goBack);
            Thread runToBottom = new Thread(bot.slide::runToBottom);
            telemetry.addData("Started slide",runToBottom);
            telemetry.update();
            runToBottom.start();
            sleep(3000);
            bot.rr.followTrajectory(strafeRight);
            switch(pipeline.getSignalVal()){//TODO figure out which one is left/center/right
                case GREEN://LEFT
                    bot.rr.followTrajectory(parkLeft);
                    break;
                case PINK://MIDDLE
                    break;
                case YELLOW://RIGHT
                    bot.rr.followTrajectory(parkRight);
                    break;
            }

            /*bot.rr.followTrajectory(strafeLeftfirst);
            switch(pipeline.getSignalVal()){
                case GREEN://LEFT
                    bot.rr.followTrajectory(parkLeft);
                    break;
                case PINK://MIDDLE
                    break;
                case YELLOW://RIGHT
                    bot.rr.followTrajectory(parkRight);
                    break;
            */
        slidePeriodic.interrupt();
        }

    }

