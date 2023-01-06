package org.firstinspires.ftc.teamcode.teleop.subsystems;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class Intake {

    private final MotorEx left, right;

    public Intake(OpMode opMode){
        left = new MotorEx(opMode.hardwareMap, "leftIntake", Motor.GoBILDA.RPM_435);
        left.setRunMode(Motor.RunMode.RawPower);
        left.setInverted(true);
        left.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT);
        right = new MotorEx(opMode.hardwareMap, "rightIntake", Motor.GoBILDA.RPM_435);
        right.setRunMode(Motor.RunMode.RawPower);
        right.setInverted(false);
        right.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT);
    }

    private void run(double speed){
        left.setInverted(true);
        right.setInverted(false);
        left.set(speed);
        right.set(speed);
    }

    public void run(){
        run(0.7);
    }

    public void spit(){
        run(-0.3);
    }

    public void stop(){
        run(0);
    }

}