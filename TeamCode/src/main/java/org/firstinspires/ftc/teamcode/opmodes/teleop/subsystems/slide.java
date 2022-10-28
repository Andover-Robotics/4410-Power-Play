package org.firstinspires.ftc.teamcode.opmodes.teleop.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.hardware.Gamepad;

public class slide {

    GamepadEx gamepad2 = new Gamepad(gamepad2);
    DcMotor slideMotor = hardwareMap.dcMotor.get("");

    public void extendSlide() {
        double y = gamepad2.right_stick_y;
        if(y > 0.0) {
            slideMotor.setPower(y);
        }
    }

    public void retractSlide() {
        double y = -gamepad2.right_stick_y;
        if(y < 0.0) {
            slideMotor.setPower(y);
        }
    }

}
