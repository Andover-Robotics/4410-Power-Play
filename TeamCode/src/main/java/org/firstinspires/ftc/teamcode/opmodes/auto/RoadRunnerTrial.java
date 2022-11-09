package org.firstinspires.ftc.teamcode.opmodes.auto;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.opmodes.auto.RoadRunnerTrials.drive.SampleMecanumDrive;


/*
     * This is an example of a more complex path to really test the tuning.
     */
    @Autonomous(group = "drive")
    public class RoadRunnerTrial extends LinearOpMode {

        private static final int X1=20;
        private static final int Y1=20;

        @Override
        public void runOpMode() throws InterruptedException {

            Pose2d startPose = new Pose2d(0, 0, 0);
            SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
            drive.setPoseEstimate(startPose);
            Trajectory traj = drive.trajectoryBuilder(new Pose2d())
                    .splineTo(new Vector2d(X1, Y1), 0)
                    .build();
            waitForStart();
            drive.followTrajectory(traj);
        }
}
