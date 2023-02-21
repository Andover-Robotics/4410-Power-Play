package org.firstinspires.ftc.teamcode.teleop.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;

@Config
public class Arm {
    private final Servo armLeft, armRight, bar;

    public static double armIntake = 0.25, armOuttake = 0.5, armStorage = 0.75, barIntake = 0.61, barOuttakeUp = 0.5, barOuttakeDown = 0.43, preload = 0.25;

    public static double armTest = 0.45, barTest = 0.4;
    public static double barStack[] = {0.6, 0.56, 0.53, 0.5, 0.45, barTest};//{0.6, 0.56, 0.53, 0.5, 0.5};
    public static double armStack[] = {0.24, 0.28, 0.33, 0.37, 0.4, armTest};//{0.4, 0.37, 0.34, 0.3, 0.25};

    public Arm(OpMode opMode){
        armLeft = opMode.hardwareMap.servo.get("armLeft");
        armRight = opMode.hardwareMap.servo.get("armRight");
        armLeft.setDirection(Servo.Direction.FORWARD);
        armRight.setDirection(Servo.Direction.FORWARD);
        bar = opMode.hardwareMap.servo.get("bar");
        bar.setDirection(Servo.Direction.FORWARD);
    }

    private void setArm(double position){
        armLeft.setPosition(position);
        armRight.setPosition(1-position);
    }

    private void setBar(double position){
        bar.setPosition(position);
    }

    public void intake(){
        setBar(barIntake);
        setArm(armIntake);
    }

    public void storage(){
        setArm(armStorage);
        setBar(barOuttakeUp);
    }

    public void preload(){
        setArm(armStorage);
        setBar(preload);
    }

    public void outtake(){
        setBar(barOuttakeUp);
        setArm(armOuttake);
    }

    public void secure() {
        setBar(barOuttakeDown);
        setArm(armOuttake);
    }

    public void intakeAuto(int i){
        barStack[5] = barTest;
        armStack[5] = armTest;
        setBar(barStack[i]);
        setArm(armStack[i]);
    }

    public void updateIntakeAuto(){
        barStack[5] = barTest;
        armStack[5] = armTest;
    }

}
