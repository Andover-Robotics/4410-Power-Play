package org.firstinspires.ftc.teamcode.teleop.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;

@Config
public class Arm {
    private final Servo armLeft, armRight, bar;

    public static double armOuttake = 0.65, armIntake = 0.28, autoArmOuttake = 0.55, armStorage = 0.75, armAutoStorage = 0.60, armAutoStorage2 = 0.75,
        barOuttake = 0.5, barSecure = 0.3, barIntake = 0.6, barAutoOuttakeUp = 0.35, autoBarOuttakeDown = 0.36, barPreload = 0.25, barAutoStorage = 0.7, barAutoStorage2 = 0.38;

    public static double armTest = 0.45, barTest = 0.4;
    public static double armStack[] = {0.28, 0.33, 0.37, 0.4, 0.44, armTest};//{0.4, 0.37, 0.34, 0.3, 0.25};
    public static double barStack[] = {0.6, 0.56, 0.52, 0.48, 0.45, barTest};//{0.6, 0.56, 0.53, 0.5, 0.5};


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
        setBar(barAutoOuttakeUp);
    }

    public void outtake(){
        setArm(armOuttake);
        setBar(barOuttake);
    }

    public void secure(){
        setArm(armOuttake);
        setBar(barSecure);
    }

    public void autoStorage(){
        setArm(armAutoStorage);
        setBar(barAutoStorage);
    }

    public void autoStorage2(){
        setArm(armAutoStorage2);
        setBar(barAutoStorage2);
    }

    public void preload(){
        setArm(armStorage);
        setBar(barPreload);
    }

    public void autoOuttake(){
        setBar(barAutoOuttakeUp);
        setArm(autoArmOuttake);
    }

    public void autoSecure() {
        setBar(autoBarOuttakeDown);
        setArm(autoArmOuttake);
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

    public void lift(double pos) {
        setBar(pos + 10);
    }

}
