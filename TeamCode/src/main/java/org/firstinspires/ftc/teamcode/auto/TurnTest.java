package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.teleop.subsystems.Bot;

/*
 * This is a simple routine to test turning capabilities.
 */
@Config
@Autonomous(name = "TurnTest", group = "drive")
public class TurnTest extends LinearOpMode {
    public static double ANGLE = 90; // deg
    Bot bot;

    @Override
    public void runOpMode() throws InterruptedException {
        bot = Bot.getInstance(this);
        waitForStart();

        if (isStopRequested()) return;

        bot.rr.turn(Math.toRadians(90));
    }
}
