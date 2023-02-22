package org.firstinspires.ftc.teamcode.auto.tester;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.auto.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.teleop.subsystems.Bot;


@Config
@Autonomous(name="'straight' test")
public class StraightTest extends LinearOpMode {

    public static double X = 52, Y = 1;

    @Override
    public void runOpMode() throws InterruptedException {
        Telemetry telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Trajectory trajectory = drive.trajectoryBuilder(new Pose2d())
                .lineTo(new Vector2d(X, Y))
                .build();

        waitForStart();

        if (isStopRequested()) return;

        drive.followTrajectory(trajectory);

        Pose2d poseEstimate = drive.getPoseEstimate();
        telemetry.addData("finalX", poseEstimate.getX());
        telemetry.addData("finalY", poseEstimate.getY());
        telemetry.addData("finalHeading", poseEstimate.getHeading()*180/Math.PI);
        telemetry.update();

        while (!isStopRequested() && opModeIsActive()) ;
    }
}
