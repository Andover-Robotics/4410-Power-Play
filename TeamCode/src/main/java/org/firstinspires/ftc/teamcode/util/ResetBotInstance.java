package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotor.RunMode;

import org.firstinspires.ftc.teamcode.teleop.subsystems.Bot;

import java.util.Map.Entry;

@Autonomous(name = "Reset Bot Instance", group = "Utility")
public class ResetBotInstance extends LinearOpMode {
    public void runOpMode() {
        Bot.instance = null;
        waitForStart();
    }
}
