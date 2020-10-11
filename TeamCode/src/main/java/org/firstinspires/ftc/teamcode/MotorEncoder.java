package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gyroscope;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@TeleOp
public class MotorEncoder extends LinearOpMode {

    public static double PI_MULTIPLE = 0.1;
    public static double F_CONSTANT = 32767;
    public static double D_CONSTANT = 0;

    public static double MAX_VELOCITY = 3060;

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
        motorR1.setDirection(DcMotorSimple.Direction.REVERSE);
        initMotor(motorR1);

        motorR2 = hardwareMap.get(DcMotorEx.class, "motorR2");
        motorR2.setDirection(DcMotorSimple.Direction.REVERSE);
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

    public static void initMotor(DcMotorEx motor)
    {
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        /*
        Your F value is calculated like this: F = 32767 / MAX_VELOCITY.

        Then your P value is calculated from your F value: P = 0.1 * F

        Then your I value is calculated from your P value: I = 0.1 * P

        Your D value should be zero.
         */
        double fValue  = F_CONSTANT / MAX_VELOCITY;
        double pValue = PI_MULTIPLE * fValue;
        double iValue = PI_MULTIPLE * pValue;

        motor.setVelocityPIDFCoefficients(pValue, iValue, D_CONSTANT, fValue);
        motor.setPositionPIDFCoefficients(5.0);
    }
}