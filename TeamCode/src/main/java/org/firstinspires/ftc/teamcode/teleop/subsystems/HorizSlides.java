package org.firstinspires.ftc.teamcode.teleop.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

@Config
public class HorizSlides {

    private final MotorEx motor;
    private final PIDFController controller;
    public static double p = 0.04, i = 0, d = 0, f = 0, staticF = 0;
    public static double tolerance = 10, powerUp = 0.1, manualDivide = 1.5, manualPower = 0;
    public static int MAXHEIGHT = 5000, fullOut = 580, fullIn = 0, outtake = 1700, intake = 2000;
    private int target = 0;

    public HorizSlides(OpMode opMode){
        motor = new MotorEx(opMode.hardwareMap, "slidesHoriz", Motor.GoBILDA.RPM_1150);
        motor.setInverted(false);
        controller = new PIDFController(p, i, d, f);
        controller.setTolerance(tolerance);
        controller.setSetPoint(target);
        motor.setRunMode(Motor.RunMode.RawPower);
        motor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
    }

    public void runTo(int t){
        motor.setInverted(false);
        controller.setSetPoint(t);
        target = t;
    }

    public void runToFullIn(){
        runTo(fullIn);
    }
    public void runToFullOut(){
        runTo(fullOut);
    }
    public void runToOuttake(){
        runTo(outtake);
    }
    public void runToIntake() {
        runTo(intake);
    }

    public void runManual(double manual){
        if(manual > 0.3 || manual < -0.3){
            manualPower = manual;
        }else{
            manualPower = 0;
        }
    }

    public void periodic(){
        controller.setPIDF(p, i, d, f);
        if(manualPower == 0) {
            if (controller.atSetPoint()) {
                motor.set(staticF);
            } else{
                motor.set(powerUp * controller.calculate(motor.getCurrentPosition()));
            }
        }else{
            controller.setSetPoint(motor.getCurrentPosition());
            motor.set(manualPower/manualDivide);
        }
    }

    public void resetEncoder(){
        motor.resetEncoder();
    }

    public int getPosition(){
        return motor.getCurrentPosition();
    }

}
