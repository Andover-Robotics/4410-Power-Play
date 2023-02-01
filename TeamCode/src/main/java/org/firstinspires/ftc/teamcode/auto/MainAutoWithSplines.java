package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.teleop.subsystems.Bot;

import java.util.Vector;

@Autonomous(name = "MainAutonomous with Splines", group = "Testing")
public class MainAutoWithSplines extends LinearOpMode {


    Bot bot;
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
        Trajectory toJunction = bot.rr.trajectoryBuilder(startPose)
                .splineTo(new Vector2d(50, 13), 0)
                .addTemporalMarker(0, () -> {
                    bot.claw.close();
                    slidePeriodic.start();
                })
                .addTemporalMarker(1, () -> {
                    bot.slide.runToTop();
                })
                .build();

        Trajectory alliance1ApproachJunction = bot.rr.trajectoryBuilder(toJunction.end())
                .forward(6)
                .addTemporalMarker(2.5, () -> {
                    bot.slide.goDown();
                    bot.claw.open();
                })
                .build();

        Trajectory alliance1GoBack = bot.rr.trajectoryBuilder(alliance1ApproachJunction.end())
                .back(4)
                .addTemporalMarker(2.5, () -> {
                    bot.slide.runTo(580);
                })
                .build();

        Trajectory toCone = bot.rr.trajectoryBuilder(alliance1GoBack.end())
                .splineTo(new Vector2d(52, 36), Math.toRadians(90))
                .addTemporalMarker(6, () -> {
                    bot.claw.close();
                })
                .build();

        Trajectory backToJunction = bot.rr.trajectoryBuilder(toCone.end())
                .splineTo(new Vector2d(52, 0), -Math.toRadians(90))
                .addDisplacementMarker(36, () -> {
                    bot.rr.turn(-Math.toRadians(90));
                })
                .build();

        if(!isRight){
            bot.rr.followTrajectory(toJunction);
        }
        else{

        }




    }
}
