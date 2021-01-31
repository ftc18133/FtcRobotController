package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

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
        double velocity = 0;

        boolean ingestMotor = false;

        while (opModeIsActive()) {
            RX = this.gamepad1.right_stick_x;
            RY = -this.gamepad1.right_stick_y;// opposite what you think
            pwr = Math.sqrt(RX*RX + RY*RY);
            velocity = pwr * CyberCatBot.MAX_VELOCITY;

            // Ingest IF
            if (this.gamepad1.x || this.gamepad1.b || this.gamepad1.a){
                if (this.gamepad1.b)
                    catbot.getRampServo().setPower(1.0);
                if (this.gamepad1.y) {
                    catbot.getRingMotor1().setPower(1);
                    catbot.getRingMotor2().setPower(1);
                }
                if (this.gamepad1.a) {
                    catbot.getIngestMotor().setPower(1);
                }
            }
            /*
            else if (this.gamepad1.a){
                // set ingest motor on if off
                if (ingestMotor) {
                    catbot.getIngestMotor().setPower(0);
                }
                else {
                    // set ingest motor off if on
                    catbot.getIngestMotor().setPower(1);
                }
                ingestMotor = !ingestMotor;
            }
             */
            else {
                catbot.getRampServo().setPower(0);
                catbot.getRingMotor1().setPower(0);
                catbot.getRingMotor2().setPower(0);
                catbot.getIngestMotor().setPower(0);
            }

            // Arm IF
            if (gamepad1.right_bumper) {
                catbot.getClawServo().setPosition(0);
                telemetry.addData("Servo: ", "Position 0");
            }
            else if (gamepad1.right_trigger != 0) {
                catbot.getClawServo().setPosition(1);
                telemetry.addData("Servo: ", "Position 1");
            }
            else if (gamepad1.left_bumper) {
                catbot.liftArm();
                telemetry.addData("Arm: ", "Arm up");
            }
            else if (gamepad1.left_trigger != 0) {
                catbot.lowerArm();
                telemetry.addData("Arm: ", "Arm down");
            }
            else {
                catbot.stopArm ();
            }

            // straight: Forward, top or bottom, push towards top or bottom (gamepad analog right) speed controlled by how far you push it
            // strafe: SIDEWAYS, left or right on gamepad analog right: Same as Straight
            // Diagonal: Diagonal, top left, top right, bottom right, bottom left: gamepad analog right: same as straight
            // Tankturn: full body turn on center, gamepad left analog: how far you push = speed

            // MOVE IF
            if (gamepad1.dpad_left){
                telemetry.addData("Direction: ", "Tank Left");
                catbot.setVelocity(-CyberCatBot.HALF_VELOCITY, -CyberCatBot.HALF_VELOCITY, CyberCatBot.HALF_VELOCITY, CyberCatBot.HALF_VELOCITY);
            }
            else if (gamepad1.dpad_right){
                telemetry.addData("Direction: ", "Tank Right");
                catbot.setVelocity(CyberCatBot.HALF_VELOCITY, CyberCatBot.HALF_VELOCITY, -CyberCatBot.HALF_VELOCITY, -CyberCatBot.HALF_VELOCITY);
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

                catbot.strafeVelocity(velocity);

            } else if ((RX < 0 && RY < 0) || (RX > 0 && RY > 0)) {

                telemetry.addData("Direction: ", "Diagonal Right");

                // opposite, switch power
                if(RX < 0 && RY < 0)
                    velocity = -velocity;

                catbot.setVelocity(velocity,0,0, velocity);

            } else if ((RX < 0 && RY > 0) || (RX > 0 && RY < 0)) {
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

            telemetry.addData("Ramp Servo", catbot.getRampServo().getPower());
            telemetry.addData("Ring Motor 1", catbot.getRingMotor1().getPower());
            telemetry.addData("Ring Motor 2", catbot.getRingMotor2().getPower());

            telemetry.update();
        }
    }
}
