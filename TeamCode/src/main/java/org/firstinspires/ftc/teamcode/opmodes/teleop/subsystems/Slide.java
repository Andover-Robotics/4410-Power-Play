package org.firstinspires.ftc.teamcode.opmodes.teleop.subsystems;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad2;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.hardware.Gamepad;

public class Slide {

    static DcMotor slideMotor = hardwareMap.dcMotor.get("slideMotor");

    public static void extendSlide() {
        double y = gamepad2.right_stick_y;
        if(y > 0.0) {
            slideMotor.setPower(y);
        }
    }

    public static void retractSlide() {
        double y = gamepad2.right_stick_y;
        if(y < 0.0) {
            slideMotor.setPower(y);
        }
    }

}
