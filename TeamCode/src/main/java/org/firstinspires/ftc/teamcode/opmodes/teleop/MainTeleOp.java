package org.firstinspires.ftc.teamcode.opmodes.teleop;

public class MainTeleOp {

    // Hi

    // Gamepad 1
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
    } else {
        bot.slide.retractSlide();
    }

    if(leftY > 0.0) {
        bot.turret.rotateLeft();
    } else {
        bot.turret.rotateRight();
    }

}
