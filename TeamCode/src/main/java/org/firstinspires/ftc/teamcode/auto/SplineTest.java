package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.teleop.subsystems.Bot;

/*
 * This is an example of a more complex path to really test the tuning.
 */
@Autonomous(name = "Spline Test", group = "drive")
public class SplineTest extends LinearOpMode {

    Bot bot;

    @Override
    public void runOpMode() throws InterruptedException {
        bot = Bot.getInstance(this);

        waitForStart();

        if (isStopRequested()) return;

        Trajectory traj = bot.rr.trajectoryBuilder(new Pose2d())
                .splineTo(new Vector2d(30, 30), 0)
                .build();

        bot.rr.followTrajectory(traj);

        sleep(2000);

        bot.rr.followTrajectory(
                bot.rr.trajectoryBuilder(traj.end(), true)
                        .splineTo(new Vector2d(0, 0), Math.toRadians(180))
                        .build()
        );
    }
}