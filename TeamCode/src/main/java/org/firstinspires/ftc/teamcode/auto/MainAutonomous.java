package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.teleop.subsystems.Bot;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvInternalCamera;

@Autonomous(name="MainAutonomousPark")
public class MainAutonomous extends LinearOpMode {

    Bot bot;
    @Override
    public void runOpMode() throws InterruptedException {
        bot = Bot.getInstance(this);

        int cameraID = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewID", "id", hardwareMap.appContext.getPackageName());
        OpenCvCamera camera = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.FRONT, cameraID);
        TestPipeline pipeline = new TestPipeline(telemetry);
        camera.setPipeline(pipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                telemetry.addLine("Camera Status: Opened");
                telemetry.update();
            }

            @Override
            public void onError(int errorCode) {
                telemetry.addData("Error code", errorCode);

            }
        });

        telemetry.addData("Current FPS:", camera.getFps());
        telemetry.addData("Current Max FPS:", camera.getCurrentPipelineMaxFps());

        waitForStart();
        if (pipeline.getSignalVal() == TestPipeline.SignalVal.GREEN) {
            telemetry.addData("The signal is green with a percentage of", TestPipeline.greenPercentage);
        }
        else if(pipeline.getSignalVal() == TestPipeline.SignalVal.PINK){
            telemetry.addData("The signal is pink with a percentage of", TestPipeline.pinkPercentage);
        }

        else if(pipeline.getSignalVal() == TestPipeline.SignalVal.YELLOW){
            telemetry.addData("The signal is yellow with a percentage of", TestPipeline.yellowPercentage);
        }
        camera.stopStreaming();

        Pose2d startPose = new Pose2d();
        Trajectory moveOut = bot.rr.trajectoryBuilder(startPose)
                .strafeLeft(2).build(),
                goToTerminal = bot.rr.trajectoryBuilder(startPose)//change this for more complex paths
                        .forward(26).build(),
                goBack = bot.rr.trajectoryBuilder(startPose)
                        .back(26).build(),
                strafeLeft = bot.rr.trajectoryBuilder(startPose)
                        .strafeLeft(26).build(),
                parkRight = bot.rr.trajectoryBuilder(startPose)
                        .forward(24).build(),
                parkLeft = bot.rr.trajectoryBuilder(startPose)
                        .back(24).build();

        switch(pipeline.getSignalVal()){//TODO figure out which one is left/center/right
            case GREEN:
                break;
            case PINK:
                break;
            case YELLOW:
                break;
        }
    }
}
