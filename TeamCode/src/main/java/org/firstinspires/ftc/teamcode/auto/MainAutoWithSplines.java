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

        int targetPos = 500;

        waitForStart();
        Thread slidePeriodic = new Thread(() -> {
            while(opModeIsActive()){
                bot.slide.periodic();
            }
        });

        Pose2d startPose = new Pose2d(0, 0, 0);
        TrajectorySequence toJunction = bot.rr.trajectorySequenceBuilder(startPose)
                .splineTo(new Vector2d(52, -5.9), -Math.toRadians(27.5))
                .addTemporalMarker(0, () -> {
                    bot.claw.close();
                    slidePeriodic.start();
                })
                .addTemporalMarker(1, () -> {
                    bot.slide.runToTop();
                })
                .addTemporalMarker(5, () -> {
                    bot.slide.goDown();
                    bot.claw.open();
                })
                .build();

        TrajectorySequence toJunctionRight = bot.rr.trajectorySequenceBuilder(startPose)
                .splineTo(new Vector2d(54, -5.9), Math.toRadians(27.5))
                .addTemporalMarker(0, () -> {
                    bot.claw.close();
                    slidePeriodic.start();
                })
                .addTemporalMarker(1, () -> {
                    bot.slide.runToTop();
                })
                .addTemporalMarker(5, () -> {
                    bot.slide.goDown();
                    bot.claw.open();
                })
                .build();

        Trajectory toCone = bot.rr.trajectoryBuilder(toJunction.end())
                .splineTo(new Vector2d(52, 20.34), Math.toRadians(27.5))
                .addTemporalMarker(6, () -> {
                    bot.claw.close();
                })
                .build();

        Trajectory toConeRight = bot.rr.trajectoryBuilder(toJunction.end())
                .splineTo(new Vector2d(54, -15.34), -Math.toRadians(27.5))
                .addTemporalMarker(6, () -> {
                    bot.claw.close();
                })
                .build();

        Trajectory backToJunction = bot.rr.trajectoryBuilder(toCone.end(), true)
                .splineTo(new Vector2d(52, -5.9), -Math.toRadians(27.5))
                .addTemporalMarker(0, () -> {
                    bot.slide.runToTop();
                })
                .addTemporalMarker(4, () -> {
                    bot.claw.open();
                })
                .build();

        Trajectory backToJunctionRight = bot.rr.trajectoryBuilder(toConeRight.end(), true)
                .splineTo(new Vector2d(54, 5.9), 0)
                .addTemporalMarker(0, () -> {
                    bot.slide.runToTop();
                })
                .addTemporalMarker(4, () -> {
                    bot.claw.open();
                })
                .build();


        if(!isRight){
            bot.rr.followTrajectorySequence(toJunction);
            for(int i = 1; i <= 5; i++) {
                int finalPos = targetPos;
                bot.rr.followTrajectory(
                        bot.rr.trajectoryBuilder(toJunction.end())
                                .splineTo(new Vector2d(52, 20.34), Math.toRadians(40))
                                .addTemporalMarker(0, () -> {
                                    bot.slide.runTo(finalPos);
                                    sleep(1000);
                                })
                                .addTemporalMarker(2, () -> {
                                    bot.claw.close();
                                    sleep(100);
                                })
                                .build()
                );
                targetPos -= 100;
                bot.rr.followTrajectory(backToJunction);
            }
        }

        if(isRight){
            bot.rr.followTrajectorySequence(toJunctionRight);
            for(int i = 1; i <= 5; i++) {
                int finalPos = targetPos;
                bot.rr.followTrajectory(
                        bot.rr.trajectoryBuilder(toJunctionRight.end())
                                .splineTo(new Vector2d(52, -18.34), -Math.toRadians(27.5))
                                .addTemporalMarker(0, () -> {
                                    bot.slide.runTo(finalPos);
                                })
                                .build()
                );
                targetPos -= 100;
                bot.rr.followTrajectory(backToJunctionRight);
            }
        }
    }
}
