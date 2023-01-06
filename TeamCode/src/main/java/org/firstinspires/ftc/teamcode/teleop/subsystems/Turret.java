package org.firstinspires.ftc.teamcode.teleop.subsystems;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad2;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.opMode;

import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.hardware.Gamepad;

public class Turret {
    private final MotorEx motor;
    private final MotorEx motor2;

    public Turret(OpMode opMode) {
        motor = new MotorEx(opMode.hardwareMap, "turret");
        motor2 = new MotorEx(opMode.hardwareMap, "turret2");
    }

    public void rotate(double leftX) {
        motor.set(leftX);
        motor2.set(leftX);
    }

}