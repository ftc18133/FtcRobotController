package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.CyberCatBot;

@TeleOp
public class MotorEncoderTestOpMode extends LinearOpMode {

    @Override
    public void runOpMode() {

        CyberCatBot catbot = new CyberCatBot(hardwareMap);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        catbot.setVelocity(1000);

        while (opModeIsActive()) {
            telemetry.addData("motor L1 velocity", catbot.getMotorL1().getVelocity());
            telemetry.addData("motor L2 velocity", catbot.getMotorL2().getVelocity());
            telemetry.addData("motor R1 velocity", catbot.getMotorR1().getVelocity());
            telemetry.addData("motor R2 velocity", catbot.getMotorR2().getVelocity());

            telemetry.addData("Status", "Running");
            telemetry.update();
        }
    }


}