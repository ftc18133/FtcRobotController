package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.Gyroscope;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@TeleOp
public class MotorEncoderJavaOpMode extends LinearOpMode {
    private DcMotorEx motorL1;
    private DcMotorEx motorL2;
    private DcMotorEx motorR1;
    private DcMotorEx motorR2;

    @Override
    public void runOpMode() {
        motorL1 = hardwareMap.get(DcMotorEx.class, "motorL1");
        initMotor(motorL1);

        motorL2 = hardwareMap.get(DcMotorEx.class, "motorL2");
        initMotor(motorL2);

        motorR1 = hardwareMap.get(DcMotorEx.class, "motorR1");
        initMotor(motorR1);

        motorR2 = hardwareMap.get(DcMotorEx.class, "motorR2");
        initMotor(motorR2);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        motorL1.setVelocity(1000);
        motorL2.setVelocity(1000);
        motorR1.setVelocity(1000);
        motorR2.setVelocity(1000);

        while (opModeIsActive()) {
            telemetry.addData("motor L1 velocity", motorL1.getVelocity());
            telemetry.addData("motor L2 velocity", motorL2.getVelocity());
            telemetry.addData("motor R1 velocity", motorR1.getVelocity());
            telemetry.addData("motor R2 velocity", motorR2.getVelocity());

            telemetry.addData("Status", "Running");
            telemetry.update();
        }
    }

    private void initMotor(DcMotorEx motor)
    {
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        /*
        Your F value is calculated like this: F = 32767 / maxV (or 2920).

        So if your max velocity is 2920 ticks per second (reasonable for a mechanism that uses the HD Hex Motor), then your F value should be about 11.2.

        Then your P value is calculated from your F value: P = 0.1 * F

        Then your I value is calculated from your P value: I = 0.1 * P

        Your D value should be zero.

        So for a maximum velocity of 2600 ticks per second, your velocity PIDF values are:
        P = 1.12
        I = 0.112
        D = 0
        F = 11.2
         */

        motor.setVelocityPIDFCoefficients(1.12, 0.112, 0, 11.2);
        motor.setPositionPIDFCoefficients(5.0);
    }
}