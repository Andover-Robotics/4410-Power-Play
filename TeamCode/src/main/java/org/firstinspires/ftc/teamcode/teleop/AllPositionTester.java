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
import org.firstinspires.ftc.teamcode.teleop.subsystems.Slides;

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

        int index = 4;

        while (opModeIsActive() && !isStopRequested()) {
            gp1.readButtons();
            gp2.readButtons();

            if (gp2.getButton(GamepadKeys.Button.A)) {
                bot.claw.open();
            } else {
                bot.claw.close();
            }

//            if(gp2.wasJustPressed(GamepadKeys.Button.START)){
//                bot.horizSlides.runToFullOut();
//                telemetry.addLine("running thing");
//            }else if(gp2.wasJustPressed(GamepadKeys.Button.BACK)){
//                bot.horizSlides.runToFullIn();
//            }

            if (gp2.getButton(GamepadKeys.Button.X)){
                bot.arm.intakeAuto(index);
            }else if(gp2.getButton(GamepadKeys.Button.Y)){
                bot.arm.outtake();
            }else if(gp2.getButton(GamepadKeys.Button.B)){
                bot.arm.secure();
            }else{
                bot.arm.storage();
            }

            if(gp1.wasJustPressed(GamepadKeys.Button.RIGHT_BUMPER)){
                index++;
                if(index > 5){
                    index = 5;
                }
            }
            if(gp1.wasJustPressed(GamepadKeys.Button.LEFT_BUMPER)){
                index--;
                if(index<0){
                    index=0;
                }
            }

            bot.arm.updateIntakeAuto();

//            if (gp2.wasJustPressed(GamepadKeys.Button.DPAD_UP)) {
////                bot.slides.runToTop();
//            }else if (gp2.wasJustPressed(GamepadKeys.Button.DPAD_LEFT)) {
////                bot.slides.runToMiddle();
//            } else if (gp2.wasJustPressed(GamepadKeys.Button.DPAD_RIGHT)) {
////                bot.slides.runToLow();
//            } else if (gp2.wasJustPressed(GamepadKeys.Button.DPAD_DOWN)) {
////                bot.slides.runToBottom();
//            }
//
//            if (gp2.wasJustPressed(GamepadKeys.Button.RIGHT_BUMPER)) {
//                bot.slides.goDown();
//            }
//
//            if (gp2.wasJustPressed(GamepadKeys.Button.LEFT_BUMPER)) {
//
//            }

//
//            if(gp2.wasJustPressed(GamepadKeys.Button.LEFT_STICK_BUTTON)){
//                bot.turret.runToFront();
//            }

            bot.horizSlides.runManual(-gp2.getRightY());
            bot.slides.runManual(-gp2.getLeftY());
            bot.turret.runManual(gp2.getLeftX());
            if(gp2.wasJustPressed(GamepadKeys.Button.RIGHT_STICK_BUTTON)){
                bot.resetEncoder();
            }
//
            bot.horizSlides.periodic();
            bot.slides.periodic();
            bot.turret.periodic();

            telemetry.addData("turret", bot.turret.getPosition());
            telemetry.addData("vert slides", bot.slides.getPosition());
            telemetry.addData("horiz slides", bot.horizSlides.getPosition());
            telemetry.addData("slides current", bot.slides.getCurrent());
            telemetry.addData("arm index", index);
            telemetry.update();

        }
    }
}
