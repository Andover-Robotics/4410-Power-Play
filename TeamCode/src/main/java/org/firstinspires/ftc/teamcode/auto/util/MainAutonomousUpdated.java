package org.firstinspires.ftc.teamcode.auto.util;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.teleop.subsystems.Bot;

@Autonomous(name = "MainAuto for V3", group = "Haha")
public class MainAutonomousUpdated extends LinearOpMode {

    boolean isRight;
    Bot bot;

    @Override
    public void runOpMode() throws InterruptedException {

        GamepadEx gp1 = new GamepadEx(gamepad1);


        while (!isStarted()) {
            if (gp1.wasJustPressed(GamepadKeys.Button.X)) {
                isRight = !isRight;
            }
            telemetry.addData("Is right side?", isRight);
            telemetry.update();
        }

        Trajectory toJunction = bot.rr.trajectoryBuilder(new Pose2d(0, 0, 0))
                .splineTo(new Vector2d(48, 13), 0)
                .addTemporalMarker(0, () -> {
                    bot.claw.close();
                    sleep(100);
                    bot.slides.runToTop();
                })
                .addTemporalMarker(5, () -> {
                    bot.slides.goDown();
                    sleep(100);
                    bot.claw.open();
                })
                .build();

        Trajectory cycle = bot.rr.trajectoryBuilder(toJunction.end())
                .splineTo(new Vector2d(48, 9), 0)
                .addTemporalMarker(0, () -> {
                    bot.intakeOut();
                    bot.storage();
                })
                .splineTo(new Vector2d(48, 13), 0)
                .addTemporalMarker(2, () -> {
                    bot.outtake();
                    sleep(100);
                    bot.claw.open();
                })
                .build();




        if(!isRight)
        {
            bot.rr.followTrajectory(toJunction);
            bot.rr.followTrajectory(cycle);
        }
        else{
            bot.rr.followTrajectory(toJunction);
        }
    }
}
