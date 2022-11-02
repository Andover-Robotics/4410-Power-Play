package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.opmodes.teleop.subsystems.Bot;
import org.firstinspires.ftc.teamcode.opmodes.teleop.subsystems.Claw;

@TeleOp(name = "MainTeleOp", group = "Competition")
public class MainTeleOp {
    @Override
    Gamepad gamepad1 = new Gamepad(gamepad1);
    Gamepad gamepad2 = new Gamepad(gamepad2);
    Claw claw = new Claw();

    Bot bot = new Bot();

    if(gamepad1.wasJustPressed(gamepad1.a)) {
        Claw.openClaw();
        if(gamepad1.a) {
            Claw.closeClaw();
            return;
        }
    }

        // Gamepad 2
        double rightY = gamepad2.right_stick_y;
        double leftY = gamepad2.left_stick_y;

    if(rightY > 0.0) {
            bot.slide.extendSlide();
        }
    else {
            bot.slide.retractSlide();
        }

//    if(leftY > 0.0) {
//            bot.turret.rotateLeft();
//        } else {
//            bot.turret.rotateRight();
//        }
//
//    }
//}
