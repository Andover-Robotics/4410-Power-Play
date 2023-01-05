package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.teleop.subsystems.Bot;

@Autonomous(name = "Reset Bot instance", group = "Utility")
public class ResetBot extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Bot.instance = null;
    }
}
