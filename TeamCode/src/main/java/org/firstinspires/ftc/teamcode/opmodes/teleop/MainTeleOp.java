//package org.firstinspires.ftc.teamcode.opmodes.teleop;
//
//import com.arcrobotics.ftclib.gamepad.GamepadEx;
//import com.arcrobotics.ftclib.gamepad.GamepadKeys;
//import com.qualcomm.robotcore.hardware.Gamepad;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//
//import org.firstinspires.ftc.teamcode.opmodes.teleop.subsystems.Bot;
//import org.firstinspires.ftc.teamcode.opmodes.teleop.subsystems.Claw;
//import org.firstinspires.ftc.teamcode.opmodes.teleop.subsystems.RealIntake;
//import org.firstinspires.ftc.teamcode.opmodes.teleop.subsystems.Slide;
//
//
//@TeleOp(name = "MainTeleOp", group = "Competition")
//public class MainTeleOp extends LinearOpMode {
//    @Override
//    public void runOpMode() throws InterruptedException {
//       GamepadEx gamepad2 = new GamepadEx(gamepad2);
//       GamepadEx gamepad1;
//       gamepad1 = new GamepadEx(gamepad1);
//
//        if((gamepad1.wasJustPressed(GamepadKeys.Button.A))) {
//           Claw.openClaw();
//            if(gamepad1.wasJustReleased(GamepadKeys.Button.A))
//            Claw.closeClaw();
//            return;
//           }
//       }
//
//       if((gamepad1.wasJustPressed(GamepadKeys.Button.X))
//        {
//            RealIntake.runIntake();
//          return;
//        }
//
//        // Gamepad 2
//        double rightY = gamepad2.right_stick_y;
//        double leftY = gamepad2.left_stick_y;
//
//       if(rightY > 0.0)
//
//        {
//            Slide.extendSlide();
//       }
//
//        else if(rightY < 0.0)
//       {
//           Slide.retractSlide();
//       }
//
////          if(leftY > 0.0) {
////            bot.turret.rotateLeft();
////      } else {////            bot.turret.rotateRight();
////       }
//
//
//    }
//
//    }
//}