package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import java.lang.Math;

/**
 * See https://docs.revrobotics.com/15mm/ftc-starter-kit-mecanum-drivetrain/mecanum-wheel-setup-and-behavior
 */

@TeleOp
public class MecanumJavaOpMode extends LinearOpMode {
    private CyberCatBot catbot;

    @Override
    public void runOpMode() {
        catbot = new CyberCatBot(hardwareMap);

        telemetry.addData("Status", "Initialized");
        telemetry.update();
        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        // run until the end of the match (driver presses STOP)

        double RX = 0;
        double RY = 0;
        double pwr = 0;
        double lpwr = 0;
        double velocity = 0;
        double LX = 0;
        double LY = 0;

        while (opModeIsActive()) {
            RX = this.gamepad1.right_stick_x;
            RY = -this.gamepad1.right_stick_y;// opposite what you think
            LX = this.gamepad1.left_stick_x;
            LY = -this.gamepad1.left_stick_y;
            pwr = Math.sqrt(RX*RX + RY*RY);
            lpwr = Math.sqrt(LX*LX + LY*LY);
            velocity = pwr * CyberCatBot.MAX_VELOCITY;

            // straight: Forward, top or bottom, push towards top or bottom (gamepad analog right) speed controlled by how far you push it
            // strafe: SIDEWAYS, left or right on gamepad analog right: Same as Straight
            // Diagonal: Diagonal, top left, top right, bottom right, bottom left: gamepad analog right: same as straight
            // Tankturn: full body turn on center, gamepad left analog: how far you push = speed

            /*
            if (this.gamepad1.a){
                catbot.getRingMotor().setPower(1.0);
            }
            else if (gamepad1.b){
                catbot.getRingMotor().setPower(0);
            }
            else if (gamepad1.x) {
                catbot.getIngestMotor().setPower(1);
            }
            else if (gamepad1.y) {
                catbot.getIngestMotor().setPower(0);
            }
            else if (LY != 0) {
                catbot.getRampMotor().setPower(lpwr);
            }
            */
            if (gamepad1.a) {
                catbot.getClawServo().setPosition(0);
                telemetry.addData("Servo: ", "Position 0");
            }
            else if (gamepad1.b) {
                catbot.getClawServo().setPosition(0.25);
                telemetry.addData("Servo: ", "Position 0.25");
            }
            
            else if (gamepad1.dpad_left){
                telemetry.addData("Direction: ", "Tank Left");
                catbot.setVelocity(-CyberCatBot.MAX_VELOCITY, -CyberCatBot.MAX_VELOCITY, CyberCatBot.MAX_VELOCITY, CyberCatBot.MAX_VELOCITY);
            }
            else if (gamepad1.dpad_right){
                telemetry.addData("Direction: ", "Tank Right");
                catbot.setVelocity(CyberCatBot.MAX_VELOCITY, CyberCatBot.MAX_VELOCITY, -CyberCatBot.MAX_VELOCITY, -CyberCatBot.MAX_VELOCITY);
            }

            else if (RX == 0 && RY == 0) {
                catbot.setVelocity(0);
                telemetry.addData("Direction: ", "No Move");
            }

            else if (RX == 0 && RY != 0) {

                if(RY>0){
                    telemetry.addData("Direction: ", "Forward");
                }else {
                    velocity=-velocity;
                    telemetry.addData("Direction: ", "Backward");
                }

                catbot.setVelocity(velocity);
            }

            else if (RY == 0 && RX != 0) {

                if (RX<0){
                    velocity=-velocity;
                    telemetry.addData("Direction: ", "Strafe Left");

                } else {
                    telemetry.addData("Direction: ", "Strafe Right");
                }

                //catbot.setVelocity(velocity, -velocity, -velocity, velocity);
                catbot.strafeVelocity(velocity);

            } else if ((RX < 0 && RY < 0) || (RX > 0 && RY > 0)) {

                telemetry.addData("Direction: ", "Diagonal Right");

                // opposite, switch power
                if(RX < 0 && RY < 0)
                    velocity = -velocity;

                catbot.setVelocity(velocity,0,0, velocity);

            } else {
                telemetry.addData("Direction: ", "Diagonal Left");

                // opposite, switch power
                if(RX > 0 && RY < 0)
                    velocity = -velocity;

                catbot.setVelocity(0, velocity, velocity, 0);
            }

            telemetry.addData("Status", "Running");
            telemetry.addData("RX", RX);
            telemetry.addData("RY", RY);
            telemetry.addData("Power", pwr);

            telemetry.addData("Motor Velocity L1", catbot.getMotorL1().getVelocity());
            telemetry.addData("Motor Velocity L2", catbot.getMotorL2().getVelocity());
            telemetry.addData("Motor Velocity R1", catbot.getMotorR1().getVelocity());
            telemetry.addData("Motor Velocity R2", catbot.getMotorR2().getVelocity());

            telemetry.update();
        }
    }
}
