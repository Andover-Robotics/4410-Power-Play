package org.firstinspires.ftc.teamcode.opmodes.teleop;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "Drive2", group = "Bruh")
public class MotorControl extends LinearOpMode {
    @Override
    public void runOpMode() {
        DcMotor motorFrontLeft = hardwareMap.dcMotor.get("motorFrontLeft");
        DcMotor motorBackLeft = hardwareMap.dcMotor.get("motorBackLeft");
        DcMotor motorFrontRight = hardwareMap.dcMotor.get("motorFrontRight");
        DcMotor motorBackRight = hardwareMap.dcMotor.get("motorBackRight");
        while(!isStopRequested()){
            double yforwardback = -1 * gamepad1.left_stick_y;
            double xforwardback = gamepad1.right_stick_x;

            double yleftright = -1 * gamepad1.right_stick_y;
            double xleftright = gamepad1.left_stick_x;

            motorFrontLeft.setPower(yforwardback+xforwardback);
            motorFrontRight.setPower(-xforwardback-yforwardback);
            motorBackLeft.setPower(xforwardback+yforwardback);
            motorBackRight.setPower(-yforwardback-xforwardback);

            motorFrontLeft.setPower(yleftright+xleftright);
            motorFrontRight.setPower(xleftright+yleftright);
            motorBackLeft.setPower(-xleftright-yleftright);
            motorBackRight.setPower(-yleftright-xleftright);
        }
    }
}