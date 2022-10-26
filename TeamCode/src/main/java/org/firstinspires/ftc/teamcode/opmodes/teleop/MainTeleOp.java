package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.hardware.Gamepad;

public class MainTeleOp {

    GamepadEx gamepad1 = new GamepadEx(gamepad1);
    GamepadEx gamepad2 = new Gamepad(gamepad2);

    if(gamepad1.a) {
            bot.claw.openClaw();
            if(gamepad1.a) {
                bot.claw.closeClaw();
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

    if(leftY > 0.0) {
            bot.turret.rotateLeft();
        } else {
            bot.turret.rotateRight();
        }

    }
}
