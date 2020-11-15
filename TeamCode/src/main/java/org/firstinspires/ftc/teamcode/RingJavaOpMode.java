package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class RingJavaOpMode extends LinearOpMode {
    private CyberCatBot catbot;

    // booleans to save motor state

    @Override
    public void runOpMode() {
        catbot = new CyberCatBot(hardwareMap);

        telemetry.addData("Status", "Initialized");
        telemetry.update();
        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        // run until the end of the match (driver presses STOP)

        while (opModeIsActive()) {

            if (this.gamepad1.a){
                // set ingest motor on if off

                // set ingest motor off if on
            }
            else if (gamepad1.b){
                // set ramp motor on if off

                // set ramp motor off if on
            }
            else if (gamepad1.x) {
                // set ring motor on if off

                // set ring motor off if on
            }

            telemetry.addData("Status", "Running");

            telemetry.addData("Ingest Motor", catbot.getRingMotor().getPower());
            telemetry.addData("Ramp Motor", catbot.getRampMotor().getPower());
            telemetry.addData("Ring Motor", catbot.getRingMotor().getPower());

            telemetry.update();
        }
    }
}
