package org.firstinspires.ftc.teamcode.opmodes.teleop.subsystems;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class realIntake {
    package org.firstinspires.ftc.teamcode.opmodes.teleop.subsystems;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;
import static hardwareMap;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.hardware.Gamepad;

    public class realintake {
        DcMotorSimple motorRight = hardwareMap.dcMotor.get("motorR");
        DcMotorSimple motorLeft = hardwareMap.dcMotor.get("motorL");

        public void runIntake() {
            motorRight.setPower(0.2);
            motorLeft.setPower(0.6);
            // hi


        }
    }
    and then this is the updated MainTeleOp.java that zach did
package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


import org.firstinspires.ftc.teamcode.opmodes.teleop.subsystems.Bot;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;
import static hardwareMap;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.hardware.Gamepad;

public class realIntake {
    DcMotorSimple motorRight = hardwareMap.dcMotor.get("motorR");
    DcMotorSimple motorLeft = hardwareMap.dcMotor.get("motorL");

            public void runIntake() {
                motorRight.setPower(0.2);
                motorLeft.setPower(0.6);
            }
        }
}
