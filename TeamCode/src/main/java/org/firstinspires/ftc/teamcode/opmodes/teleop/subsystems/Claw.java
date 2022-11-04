package org.firstinspires.ftc.teamcode.opmodes.teleop.subsystems;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class Claw {
    public static SimpleServo servoClaw1 = new SimpleServo(hardwareMap, "servo1", 0, 90) {};
    public static SimpleServo servoClaw2 = new SimpleServo(hardwareMap, "servo2", 0, 90) {
        @Override
        public void rotateByAngle(double angle, AngleUnit angleUnit) {

        }

        @Override
        public void rotateByAngle(double degrees) {

        }

        @Override
        public void turnToAngle(double angle, AngleUnit angleUnit) {

        }

        @Override
        public void turnToAngle(double degrees) {

        }

        @Override
        public void rotateBy(double position) {

        }

        @Override
        public void setPosition(double position) {

        }

        @Override
        public void setRange(double min, double max, AngleUnit angleUnit) {

        }

        @Override
        public void setRange(double min, double max) {

        }

        @Override
        public void setInverted(boolean isInverted) {

        }

        @Override
        public boolean getInverted() {
            return false;
        }

        @Override
        public double getPosition() {
            return 0;
        }

        @Override
        public double getAngle(AngleUnit angleUnit) {
            return 0;
        }

        @Override
        public double getAngle() {
            return 0;
        }

        @Override
        public void disable() {

        }

        @Override
        public String getDeviceType() {
            return null;
        }
    };

    public static void main(String[] args) {
        double degreeRange1 = servoClaw1.getAngleRange();
        double degreeRange2 = servoClaw2.getAngleRange();

        servoClaw2.setInverted(true);
        servoClaw1.setPosition(45);
        servoClaw2.setPosition(45);
    }


    public static void openClaw() {
        servoClaw1.turnToAngle(45);
        servoClaw2.turnToAngle(45);
    }
    public static void closeClaw() {
        servoClaw1.turnToAngle(90);
        servoClaw2.turnToAngle(90);
    }
}
