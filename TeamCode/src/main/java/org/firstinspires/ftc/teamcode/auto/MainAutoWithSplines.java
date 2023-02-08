package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.auto.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.teleop.subsystems.Bot;

import java.util.Vector;

@Autonomous(name = "MainAutonomous with Splines", group = "Testing")
public class MainAutoWithSplines extends LinearOpMode {


    Bot bot;
    //hi
    boolean isRight;

    @Override
    public void runOpMode() throws InterruptedException {

        GamepadEx gp1 = new GamepadEx(gamepad1);
        bot = Bot.getInstance(this);

        while(!isStarted()){
            if(gp1.wasJustPressed(GamepadKeys.Button.X))
                isRight = !isRight;
        }
        telemetry.addData("Is right side?", isRight);
        telemetry.update();

        waitForStart();
        Thread slidePeriodic = new Thread(() -> {
            while(opModeIsActive()){
                bot.slide.periodic();
            }
        });

        Pose2d startPose = new Pose2d(0, 0, 0);
        TrajectorySequence toJunction = bot.rr.trajectorySequenceBuilder(startPose)
                .splineTo(new Vector2d(54, -5.9), -Math.toRadians(27.5))
                .addTemporalMarker(0, () -> {
                    bot.claw.close();
                    slidePeriodic.start();
                })
                .addTemporalMarker(1, () -> {
                    bot.slide.runToTop();
                })
                .addTemporalMarker(8, () -> {
                    bot.slide.goDown();
                    bot.claw.open();
                })
                .build();

        Trajectory toCone = bot.rr.trajectoryBuilder(toJunction.end())
                .splineTo(new Vector2d(65.6, -12.34), Math.toRadians(90))
                .addTemporalMarker(6, () -> {
                    bot.claw.close();
                })
                .build();

        Trajectory backToJunction = bot.rr.trajectoryBuilder(toCone.end())
                .splineTo(new Vector2d(23.31, -23), Math.toRadians(95.44))
                .addTemporalMarker(5, () -> {
                    bot.slide.runToTop();
                })
                .build();

        TrajectorySequence test = bot.rr.trajectorySequenceBuilder(new Pose2d(-36.34, -71.09, Math.toRadians(90.00)))
                .splineTo(new Vector2d(-9.14, -40.69), Math.toRadians(75.48))
                .splineTo(new Vector2d(-24.23, -4.11), Math.toRadians(79.08))
                .splineTo(new Vector2d(-65.60, -12.34), Math.toRadians(190.53))
                .build();


        if(!isRight){
            bot.rr.followTrajectorySequence(toJunction);

        }
    }
}
