package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class RingJavaOpMode extends LinearOpMode {
    private CyberCatBot catbot;

    // booleans to save motor state
    boolean ingestMotor;
    boolean rampMotor;
    boolean ringMotor;

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
                if (ingestMotor) {
                    catbot.getIngestMotor().setPower(0);
                }
                else {
                    // set ingest motor off if on
                    catbot.getIngestMotor().setPower(1);
                }
                ingestMotor = !ingestMotor;
            }
            else if (gamepad1.b){
                if (rampMotor) {
                    // set ramp motor on if off
                    catbot.getRampMotor().setPower(0);
                }
                    else {
                    // set ramp motor off if on
                    catbot.getRampMotor().setPower(1);
                }
                rampMotor = !rampMotor;
            }
            else if (gamepad1.x) {
                if (ringMotor) {
                    // set ring motor on if off
                    catbot.getRingMotor().setPower(0);
                }
                else {
                    // set ring motor off if on
                    catbot.getRingMotor().setPower(1);
                }
                ringMotor = !ringMotor;
            }

            telemetry.addData("Status", "Running");

            telemetry.addData("Ingest Motor", catbot.getRingMotor().getPower());
            telemetry.addData("Ramp Motor", catbot.getRampMotor().getPower());
            telemetry.addData("Ring Motor", catbot.getRingMotor().getPower());

            telemetry.update();
        }
    }
}
