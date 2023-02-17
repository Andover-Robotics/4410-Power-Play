package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.teleop.subsystems.Bot;

@TeleOp(name="all position tester")
public class AllPositionTester extends LinearOpMode {

    private Bot bot;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        bot = Bot.getInstance(this);
        GamepadEx gp2 = new GamepadEx(gamepad2);
        GamepadEx gp1 = new GamepadEx(gamepad1);

         bot.initializeImus();

        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {
            gp1.readButtons();
            gp2.readButtons();

            if (gp2.getButton(GamepadKeys.Button.A)) {
                bot.claw.open();
            } else {
                bot.claw.close();
            }

            if(gp2.wasJustPressed(GamepadKeys.Button.Y)){
                bot.horizSlides.runToFullOut();
            }else if(gp2.wasJustPressed(GamepadKeys.Button.B)){
                bot.horizSlides.runToFullIn();
            }

            if (gp2.getButton(GamepadKeys.Button.X)){
                bot.arm.storage();
            }else if(gp2.getButton(GamepadKeys.Button.RIGHT_STICK_BUTTON)){
                bot.arm.outtake();
            }else if(gp2.getButton(GamepadKeys.Button.LEFT_STICK_BUTTON)){
                bot.arm.secure();
            }else{
                bot.arm.intake();
            }

            if (gp2.wasJustPressed(GamepadKeys.Button.DPAD_UP)) {
                bot.slides.runToTop();
            }else if (gp2.wasJustPressed(GamepadKeys.Button.DPAD_LEFT)) {
                bot.slides.runToMiddle();
            } else if (gp2.wasJustPressed(GamepadKeys.Button.DPAD_RIGHT)) {
                bot.slides.runToLow();
            } else if (gp2.wasJustPressed(GamepadKeys.Button.DPAD_DOWN)) {
                bot.slides.runToBottom();
            }

            if (gp2.wasJustPressed(GamepadKeys.Button.RIGHT_BUMPER)) {
                bot.slides.goDown();
            }

            if (gp2.wasJustPressed(GamepadKeys.Button.LEFT_BUMPER)) {
                bot.slides.goUp();
            }



            bot.horizSlides.runManual(-gp2.getRightY());
            bot.slides.runManual(-gp2.getLeftY());
            bot.turret.runManual(gp2.getLeftX());
            if(gp2.wasJustPressed(GamepadKeys.Button.START)){
                bot.resetEncoder();
            }

            bot.horizSlides.periodic();
            bot.slides.periodic();
            bot.turret.periodic();

            telemetry.addData("turret", bot.turret.getPosition());
            telemetry.addData("vert slides", bot.slides.getPosition());
            telemetry.addData("horiz slides", bot.horizSlides.getPosition());
            telemetry.update();

        }
    }
    private double getIMU(){
        return (bot.imu0.getAngularOrientation().toAngleUnit(AngleUnit.RADIANS).firstAngle + bot.imu1.getAngularOrientation().toAngleUnit(AngleUnit.RADIANS).firstAngle) / 2;
    }
}
