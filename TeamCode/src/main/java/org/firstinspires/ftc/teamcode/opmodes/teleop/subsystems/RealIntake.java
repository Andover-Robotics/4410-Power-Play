package org.firstinspires.ftc.teamcode.opmodes.teleop.subsystems;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.qualcomm.robotcore.hardware.DcMotorSimple;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.hardware.Gamepad;

    public class RealIntake {
        DcMotorSimple motorRight = hardwareMap.dcMotor.get("motorR");
        DcMotorSimple motorLeft = hardwareMap.dcMotor.get("motorL");

        public static void runIntake() {
            motorRight.setPower(0.2);
            motorLeft.setPower(0.6);

        }
    }