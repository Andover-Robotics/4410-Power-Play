package org.firstinspires.ftc.teamcode.opmodes.teleop.subsystems;
package com.arcrobotics.ftclib.hardware;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class Claw {
    public static ServoEx servoClaw1 = new SimpleServo(hardwareMap, "servo1", 0, 90);
    public static ServoEx servoClaw2 = new SimpleServo(hardwareMap, "servo2", 0, 90);
    public static void main(String[] args) {
        double degreeRange1 = servoClaw1.getAngleRange();
        double degreeRange2 = servoClaw2.getAngleRange();

        servoClaw2.setInverted(true);
        servoClaw1.setPosition(45);
        servoClaw2.setPosition(45);
    }


    public static boolean openClaw() {
        servoClaw1.turnToAngle(45);
        servoClaw2.turnToAngle(45);
    }
    public static boolean closeClaw() {
        servoClaw1.turnToAngle(90);
        servoClaw2.turnToAngle(90);
    }
}
