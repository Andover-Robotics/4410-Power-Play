//package org.firstinspires.ftc.teamcode.opmodes.teleop.subsystems;
//
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.arcrobotics.ftclib.gamepad.GamepadEx;
//import com.qualcomm.robotcore.hardware.Gamepad;
//import com.qualcomm.robotcore.hardware.HardwareMap;
//
//public class slide {
//
//    Gamepad gamepad2 = new Gamepad(gamepad2);
//    public HardwareMap hardwareMap;
//    DcMotor slideMotor = hardwareMap.dcMotor.get("slideMotor");
//
//    public void extendSlide() {
//        double y = gamepad2.right_stick_y;
//        if(y > 0.0) {
//            slideMotor.setPower(y);
//        }
//    }
//
//    public void retractSlide() {
//        double y = gamepad2.right_stick_y;
//        if(y < 0.0) {
//            slideMotor.setPower(y);
//        }
//    }
//
//}
