package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;

import java.util.List;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;

/**
 * This file illustrates the concept of driving a path based on encoder counts.
 * The code is structured as a LinearOpMode
 *
 * The code REQUIRES that you DO have encoders on the wheels,
 *
 *  This methods assumes that each movement is relative to the last stopping place.
 *  There are other ways to perform encoder based moves, but this method is probably the simplest.
 *  This code uses the RUN_TO_POSITION mode to enable the Motor controllers to generate the run profile
 */

@Autonomous
public class AutonomousJavaOpMode extends LinearOpMode {

    // constants ***********************************************************************************

    // eg: HD Hex Motor https://docs.revrobotics.com/rev-control-system/sensors/encoders/motor-based-encoders
    private static final double     COUNTS_PER_MOTOR_REV    = 28 ;
    private static final double     DRIVE_GEAR_REDUCTION    = 20 ;     // 20x gear box
    private static final double     WHEEL_DIAMETER_CM       = 7.5 ;     // For figuring circumference
    private static final double     COUNTS_PER_CM           = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_CM * 3.1415);
    private static final double     INCHES_TO_CM            = 2.54;

    private static final double AUTONOMOUS_VELOCITY = 1500;
    private static final double ZERO_VELOCITY = 0;

    private static final int SQUARE_A = 0;
    private static final int SQUARE_B = 1;
    private static final int SQUARE_C = 2;
    private static final double SQUARE_WIDTH = 22.75;

    // instance vars *******************************************************************************

    /* Declare OpMode members. */
    private CyberCatBot catbot;
    //private ElapsedTime     runtime = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {

        /*
         * Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */

        catbot = new CyberCatBot(hardwareMap);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // From YouTube video: https://www.youtube.com/watch?v=H3V3A7CgwPU

        // Deliver Wobble Goal to correct target zone (15 points)
        // pick the correct square to go to based on the number of rings
        // 0 rings == A
        // 1 ring == B
        // 4 rings == C


        int square = detectSquare();

        telemetry.addData("Square", square);
        telemetry.update();

        // drive to the squares based on relative position

        driveToSquare(square);

        // TODO: Fix Wobble Goal Placement direction
        // place the wobble goal completely into the square
        // placeWobbleGoal();

        // Move robot behind line
        moveToLaunch(square);

        // Place ring in low goal (3 points ea.)
        // drive to goal
        // place 1 into low goal
        // Ring launched into mid goal (6 points ea.)
        // drive to behind launch line
        // shoot 2 rings into mid goal
        // Ring launched into high goal (12 points ea.)
        // Knock down Power Shot Target (15 points ea.)

        // TODO: Rings falling off ramp
        fireRings();

        // Park over launch line (5 points)
        //stopAtLine();

    }

    private int detectSquare()
    {
        int square = SQUARE_A;
        go(AUTONOMOUS_VELOCITY, 0.5*SQUARE_WIDTH*INCHES_TO_CM, CyberCatBot.FORWARD);


        if (catbot.getTfod() != null) {
            catbot.getTfod().activate();
        }

        if (opModeIsActive()) {
            // break after 2s
            long start = System.currentTimeMillis();
            long current = -1l;
            // break after 5s if doesn't detect rings
            while (opModeIsActive() && (current - start) < 5000) {
                if (catbot.getTfod() != null) {
                    // getUpdatedRecognitions() will return null if no new information is available since
                    // the last time that call was made.
                    List<Recognition> updatedRecognitions = catbot.getTfod().getUpdatedRecognitions();
                    if (updatedRecognitions != null) {
                        telemetry.addData("# Object Detected", updatedRecognitions.size());
                        // step through the list of recognitions and display boundary info.
                        int i = 0;
                        for (Recognition recognition : updatedRecognitions) {
                            telemetry.addData(String.format("label (%d)", i), recognition.getLabel());
                            if (recognition.getLabel().equals(catbot.LABEL_SINGLE)) {
                                square = SQUARE_B;
                                break;
                            }
                            else if (recognition.getLabel().equals(catbot.LABEL_QUAD)) {
                                square = SQUARE_C;
                                break;
                            }
                        }
                        telemetry.update();
                    }
                }
                current = System.currentTimeMillis();
            }
        }

        if (catbot.getTfod() != null) {
            catbot.getTfod().shutdown();
        }

        return square;
    }

    private void driveToSquare(int square)
    {
        //getCatbotLocation();

        //if (catbot.getLastLocation() != null){
            if (square == SQUARE_A) {
                go(AUTONOMOUS_VELOCITY, (80*INCHES_TO_CM) - (1.25*SQUARE_WIDTH*INCHES_TO_CM), CyberCatBot.FORWARD);
                go(AUTONOMOUS_VELOCITY, 1.5*SQUARE_WIDTH*INCHES_TO_CM, CyberCatBot.RIGHT);
            }
            else if (square == SQUARE_B) {
                go(AUTONOMOUS_VELOCITY, 102.75*INCHES_TO_CM - (0.5*SQUARE_WIDTH*INCHES_TO_CM), CyberCatBot.FORWARD);
                go(AUTONOMOUS_VELOCITY, 11.375*INCHES_TO_CM, CyberCatBot.LEFT);
            }
            else if (square == SQUARE_C) {
                go(AUTONOMOUS_VELOCITY, 125.5*INCHES_TO_CM - (0.5*SQUARE_WIDTH*INCHES_TO_CM), CyberCatBot.FORWARD);
                go(AUTONOMOUS_VELOCITY, 0.5*SQUARE_WIDTH*INCHES_TO_CM, CyberCatBot.RIGHT);
            }

        //}

    }

    private void getCatbotLocation()
    {
        catbot.getTargetsUltimateGoal().activate();

        // check all the trackable targets to see which one (if any) is visible.
        while (catbot.getLastLocation() == null) {

            boolean targetVisible = false;
            for (VuforiaTrackable trackable : catbot.getAllTrackables()) {
                if (((VuforiaTrackableDefaultListener) trackable.getListener()).isVisible()) {
                    telemetry.addData("Visible Target", trackable.getName());
                    targetVisible = true;

                    // getUpdatedRobotLocation() will return null if no new information is available since
                    // the last time that call was made, or if the trackable is not currently visible.
                    OpenGLMatrix robotLocationTransform = ((VuforiaTrackableDefaultListener) trackable.getListener()).getUpdatedRobotLocation();
                    if (robotLocationTransform != null) {
                        catbot.setLastLocation(robotLocationTransform);
                    }
                    break;
                }
            }

            // Provide feedback as to where the robot is located (if we know).
            if (targetVisible) {
                // express position (translation) of robot in inches.
                VectorF translation = catbot.getLastLocation().getTranslation();
                telemetry.addData("Pos (in)", "{X, Y, Z} = %.1f, %.1f, %.1f",
                        translation.get(0) / catbot.mmPerInch, translation.get(1) / catbot.mmPerInch, translation.get(2) / catbot.mmPerInch);

                // express the rotation of the robot in degrees.
                Orientation rotation = Orientation.getOrientation(catbot.getLastLocation(), EXTRINSIC, XYZ, DEGREES);
                telemetry.addData("Rot (deg)", "{Roll, Pitch, Heading} = %.0f, %.0f, %.0f", rotation.firstAngle, rotation.secondAngle, rotation.thirdAngle);
            } else {
                telemetry.addData("Visible Target", "none");
            }
            telemetry.update();
        }

        catbot.getTargetsUltimateGoal().deactivate();
    }

    private void stopAtLine()
    {
        // Wait for the game to start (driver presses PLAY)
        // Abort this loop is started or stopped.
        while (!(isStarted() || isStopRequested())) {

            // Display the light level while we are waiting to start
            telemetry.addData("Light Level", catbot.getLightSensor().alpha());
            telemetry.update();
            idle();
        }

        // Start the robot moving forward, and then begin looking for a white line.
        catbot.setVelocity(AUTONOMOUS_VELOCITY);
        // run until the white line is seen OR the driver presses STOP;
        while (opModeIsActive() && (catbot.getLightSensor().alpha() < catbot.WHITE_THRESHOLD)) {

            // Display the light level while we are looking for the line
            telemetry.addData("Light Level",  catbot.getLightSensor().alpha());
            //telemetry.addData("Light Level",  lightSensor.getLightDetected());
            telemetry.update();
        }
        // Stop all motors
        catbot.setVelocity(0);
    }

    private void go(double velocity,
                    double distance,
                    int direction)
    {
        if (opModeIsActive()) {

            int newLeft1Target = 0;
            int newLeft2Target = 0;
            int newRight1Target = 0;
            int newRight2Target = 0;

            // Determine new target position, and pass to motor controller
            if(direction == CyberCatBot.FORWARD) {
                newLeft1Target = catbot.getMotorL1().getCurrentPosition() + (int) (distance * COUNTS_PER_CM);
                newLeft2Target = catbot.getMotorL2().getCurrentPosition() + (int) (distance * COUNTS_PER_CM);
                newRight1Target = catbot.getMotorR1().getCurrentPosition() + (int) (distance * COUNTS_PER_CM);
                newRight2Target = catbot.getMotorR2().getCurrentPosition() + (int) (distance * COUNTS_PER_CM);
            }
            else if (direction == CyberCatBot.BACKWARD)
            {
                velocity = -velocity;
                newLeft1Target = catbot.getMotorL1().getCurrentPosition() - (int) (distance * COUNTS_PER_CM);
                newLeft2Target = catbot.getMotorL2().getCurrentPosition() - (int) (distance * COUNTS_PER_CM);
                newRight1Target = catbot.getMotorR1().getCurrentPosition() - (int) (distance * COUNTS_PER_CM);
                newRight2Target = catbot.getMotorR2().getCurrentPosition() - (int) (distance * COUNTS_PER_CM);
            }
            else if (direction == CyberCatBot.RIGHT)
            {
                newLeft1Target = catbot.getMotorL1().getCurrentPosition() + (int) (distance * COUNTS_PER_CM);
                newLeft2Target = catbot.getMotorL2().getCurrentPosition() - (int) (distance * COUNTS_PER_CM);
                newRight1Target = catbot.getMotorR1().getCurrentPosition() - (int) (distance * COUNTS_PER_CM);
                newRight2Target = catbot.getMotorR2().getCurrentPosition() + (int) (distance * COUNTS_PER_CM);
            }
            else if (direction == CyberCatBot.LEFT)
            {
                newLeft1Target = catbot.getMotorL1().getCurrentPosition() - (int) (distance * COUNTS_PER_CM);
                newLeft2Target = catbot.getMotorL2().getCurrentPosition() + (int) (distance * COUNTS_PER_CM);
                newRight1Target = catbot.getMotorR1().getCurrentPosition() + (int) (distance * COUNTS_PER_CM);
                newRight2Target = catbot.getMotorR2().getCurrentPosition() - (int) (distance * COUNTS_PER_CM);
            }
            else
            {
                telemetry.addData("Direction", "Invalid");
                telemetry.update();
                return;
            }

            catbot.getMotorL1().setTargetPosition(newLeft1Target);
            catbot.getMotorR1().setTargetPosition(newRight1Target);
            catbot.getMotorL2().setTargetPosition(newLeft2Target);
            catbot.getMotorR2().setTargetPosition(newRight2Target);

            // Turn On RUN_TO_POSITION
            catbot.setMotorMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            if (direction == CyberCatBot.FORWARD || direction == CyberCatBot.BACKWARD) {
                catbot.setVelocity(velocity);
            }
            else if (direction == CyberCatBot.RIGHT) {
                catbot.setVelocity(velocity, -velocity, -velocity, velocity);
            }
                else if (direction == CyberCatBot.LEFT) {
                    catbot.setVelocity(-velocity, velocity, velocity, -velocity);
            }


            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (catbot.getMotorL1().isBusy() && catbot.getMotorR1().isBusy() &&
                     catbot.getMotorL2().isBusy() && catbot.getMotorR2().isBusy())) {

                // Display it for the driver.
                telemetry.addData("Path1", "Running to %7d : %7d : %7d : %7d",
                        newLeft1Target, newLeft2Target, newRight1Target, newRight2Target);
                telemetry.addData("Path2", "Running at %7d :%7d : %7d : %7d",
                        catbot.getMotorL1().getCurrentPosition(),
                        catbot.getMotorL2().getCurrentPosition(),
                        catbot.getMotorR1().getCurrentPosition(),
                        catbot.getMotorR2().getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            catbot.setVelocity(ZERO_VELOCITY);

            // Turn off RUN_TO_POSITION
            catbot.setMotorMode(DcMotor.RunMode.RUN_USING_ENCODER);
            sleep(250);   // optional pause after each move

        }


    }

     private void moveToLaunch(int square){
         if (square == SQUARE_A) {
             go(AUTONOMOUS_VELOCITY, 22.75*INCHES_TO_CM, CyberCatBot.LEFT);
             go(AUTONOMOUS_VELOCITY, 22.75*INCHES_TO_CM, CyberCatBot.BACKWARD);
         }
         else if (square == SQUARE_B){
             go(AUTONOMOUS_VELOCITY, 2*22.75*INCHES_TO_CM, CyberCatBot.BACKWARD);
         }
         else {
             go(AUTONOMOUS_VELOCITY, 3*22.75*INCHES_TO_CM, CyberCatBot.LEFT);
             go(AUTONOMOUS_VELOCITY, 3*22.75*INCHES_TO_CM, CyberCatBot.BACKWARD);
         }
     }
    private void fireRings() {

        catbot.getRampServo().setPower(1);
        catbot.getRingMotor1().setPower(1);
        catbot.getRingMotor2().setPower(1);
        sleep(7000);
        catbot.getRingMotor1().setPower(0);
        catbot.getRingMotor2().setPower(0);
        catbot.getRampServo().setPower(0);
    }
    private void placeWobbleGoal () {
        catbot.lowerArm();
        sleep (1000);
        catbot.stopArm();
        //open claw
        catbot.getClawServo().setPosition(1);
        catbot.liftArm();
        sleep (1000);
        catbot.stopArm();
    }
}
