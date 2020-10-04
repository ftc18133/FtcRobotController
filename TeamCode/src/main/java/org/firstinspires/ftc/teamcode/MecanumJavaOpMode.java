package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.Gyroscope;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import java.lang.Math;

/**
 * See https://docs.revrobotics.com/15mm/ftc-starter-kit-mecanum-drivetrain/mecanum-wheel-setup-and-behavior
 */

@TeleOp
public class MecanumJavaOpMode extends LinearOpMode {
    private DcMotor motorL1;
    private DcMotor motorL2;
    private DcMotor motorR1;
    private DcMotor motorR2;

    @Override
    public void runOpMode() {
        motorL1 = hardwareMap.get(DcMotor.class, "motorL1");//port: 0
        motorL2 = hardwareMap.get(DcMotor.class, "motorL2");//port: 1
        motorR1 = hardwareMap.get(DcMotor.class, "motorR1");//port: 2
        motorR2 = hardwareMap.get(DcMotor.class, "motorR2");//port: 3

        telemetry.addData("Status", "Initialized");
        telemetry.update();
        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        // run until the end of the match (driver presses STOP)

        double tgtPower = 0;
        double LX = 0;
        double LY = 0;
        double RX = 0;
        double RY = 0;
        double pwr = 0;
        double pwrpivot = 0;

        while (opModeIsActive()) {
            //tgtPower = -this.gamepad1.right_stick_y;
            LX = this.gamepad1.left_stick_x;
            LY = this.gamepad1.left_stick_y;
            RX = this.gamepad1.right_stick_x;
            RY = this.gamepad1.right_stick_y;
            pwr = Math.sqrt(RX*RX + RY*RY);
            pwrpivot = Math.sqrt(LX*LX + LY*LY);


            //tgtPower = -this.gamepad1.right_stick_y;
            // straight: Forward, top or bottom, push towards top or bottom (gamepad analog right) speed controlled by how far you push it
            // strafe: SIDEWAYS, left or right on gamepad analog right: Same as Straight
            //Diagonal: Diagonal, top left, top right, bottom right, bottom left: gamepad analog right: same as straight
            //Tankturn: full body turn on center, gamepad left analog: how far you push = speed
            if (RX == 0 && RY ==0) {
                motorL1.setPower(0);
                motorL2.setPower(0);
                motorR1.setPower(0);
                motorR2.setPower(0);
                telemetry.addData("Direction: ", "No Move");

            }
            else if (RX == 0 && RY != 0) {
                if(RY<0){
                    pwr=-1*pwr;
                }
                motorL1.setPower(pwr);
                motorL2.setPower(pwr);
                motorR1.setPower(-1*pwr); //negative because it's on the right side
                motorR2.setPower(-1*pwr); //negative because it's on the right side
                telemetry.addData("Direction: ", "Straight");


                //foreward and back
            } else if (RY == 0 && RX != 0) {
                if (RX<0){
                    pwr=-1*pwr;
                }
                motorL1.setPower(pwr);
                motorL2.setPower(-1*pwr); //negative because it's on the left side
                motorR1.setPower(pwr);
                motorR2.setPower(-1*pwr); //negative because it's on the left side
                telemetry.addData("Direction: ", "Strafe");

                //sideways
            } else if ((RX>0 && RY >0) || (RX <0 && RY < 0)) {
                motorL1.setPower(-1*pwr);
                motorL2.setPower(0);
                motorR1.setPower(0);
                motorR2.setPower(-1*pwr);  //negative because it's on the left side
                telemetry.addData("Direction: ", "Diagonal");

                //diagonal up right
            } else {
                motorL1.setPower(0);
                motorL2.setPower(pwr); //negative because it's on the left side
                motorR1.setPower(-1*pwr);
                motorR2.setPower(0);
                telemetry.addData("Direction: ", "Diagonal");

                //diagonal up left
            }

            telemetry.addData("Status", "Running");
            telemetry.addData("RX",RX);
            telemetry.addData( "RY", RY);
            telemetry.addData("Power", pwr);

            // telemetry.addData("Direction Power", pwr);
            telemetry.addData("Motor Power L1", motorL1.getPower());
            telemetry.addData("Motor Power L2", motorL2.getPower());
            telemetry.addData("Motor Power R1", motorR1.getPower());
            telemetry.addData("Motor Power R2", motorR2.getPower());

            telemetry.update();
        }
    }
}
