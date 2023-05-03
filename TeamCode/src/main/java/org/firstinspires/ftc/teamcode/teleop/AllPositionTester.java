package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.teleop.subsystems.Bot;
import org.firstinspires.ftc.teamcode.teleop.subsystems.HorizSlides;

@TeleOp(name="all position tester")
public class AllPositionTester extends LinearOpMode {

    private Bot bot;

    private double turretslidespeed = 1;

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
                bot.arm.autoOuttake();
            }else if(gp2.getButton(GamepadKeys.Button.B)){
                bot.arm.autoSecure();
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

            double rightX = gp2.getRightX(), leftY = gp2.getLeftY();

            bot.horizSlides.runManual(leftY * Math.abs(leftY) * turretslidespeed);
            //bot.slides.runManual(-gp2.getRightY());
            bot.turret.runManual(rightX * Math.abs(rightX) * turretslidespeed / (1 + bot.horizSlides.getPosition() / 580.0));
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
            telemetry.addData("vert slides current", bot.slides.getCurrent());
            telemetry.addData("horiz slides current", bot.horizSlides.getCurrent());
            telemetry.addData("horiz slides power", bot.horizSlides.manualPower);
            telemetry.addData("arm index", index);
            telemetry.update();

        }
    }
}
