package org.firstinspires.ftc.teamcode.test;
// import lines were omitted. OnBotJava will add them automatically.

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;


@TeleOp
public class RunToPositionTestOpMode extends LinearOpMode {
    DcMotorEx motor;

    @Override
    public void runOpMode() {

        //motor = hardwareMap.get(DcMotorEx.class, "motorArm");
        //motor.setDirection(DcMotorSimple.Direction.REVERSE);

        // Reset the encoder during initialization
        //motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        /*
        Your F value is calculated like this: F = 32767 / MAX_VELOCITY.

        Then your P value is calculated from your F value: P = 0.1 * F

        Then your I value is calculated from your P value: I = 0.1 * P

        Your D value should be zero.
         */
        /*
        double fValue = MotorEncoder.F_CONSTANT / MotorEncoder.MAX_VELOCITY;
        double pValue = MotorEncoder.PI_MULTIPLE * fValue;
        double iValue = MotorEncoder.PI_MULTIPLE * pValue;

        motor.setVelocityPIDFCoefficients(pValue, iValue, MotorEncoder.D_CONSTANT, fValue);
        motor.setPositionPIDFCoefficients(MotorEncoder.POSITION_CONSTANT);

        waitForStart();

        // Set the motor's target position to 300 ticks
        motor.setTargetPosition(50);

        // Switch to RUN_TO_POSITION mode
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Start the motor moving by setting the max velocity to 200 ticks per second
        motor.setVelocity(200);

        // While the Op Mode is running, show the motor's status via telemetry
        while (opModeIsActive()) {
            telemetry.addData("velocity", motor.getVelocity());
            telemetry.addData("position", motor.getCurrentPosition());
            telemetry.addData("is at target", !motor.isBusy());
            telemetry.update();
        }
        */
    }
}