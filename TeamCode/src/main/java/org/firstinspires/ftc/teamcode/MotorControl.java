package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

public class MotorControl extends LinearOpMode{
    @Override
    public void runOpMode() throws InterruptedException{
        DcMotor motorFrontLeft = hardwareMap.dcMotor.get("motorFrontLeft");
        DcMotor motorBackLeft = hardwareMap.dcMotor.get("motorBackLeft");
        DcMotor motorFrontRight = hardwareMap.dcMotor.get("motorFrontRight");
        DcMotor motorBackRight = hardwareMap.dcMotor.get("motorBackRight");

        double y = -1 * gamepad1.left_stick_y;
        double x = gamepad1.right_stick_x;
        motorFrontLeft.setPower(y+x);
        motorBackLeft.setPower(y-x);
        motorFrontRight.setPower(y-x);
        motorBackRight.setPower(y+x);
    }
}