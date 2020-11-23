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

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;

/**
 * This file illustrates the concept of driving a path based on encoder counts.
 * It uses the common Pushbot hardware class to define the drive on the robot.
 * The code is structured as a LinearOpMode
 *
 * The code REQUIRES that you DO have encoders on the wheels,
 *   otherwise you would use: PushbotAutoDriveByTime;
 *
 *  This code ALSO requires that the drive Motors have been configured such that a positive
 *  power command moves them forwards, and causes the encoders to count UP.
 *
 *   The desired path in this example is:
 *   - Drive forward for 10 centimeters
 *   - Spin right for 10 centimeters
 *   - Drive Backwards for 10 centimeters
 *
 *  The code is written using a method called: encoderDrive(speed, leftInches, rightInches, timeoutS)
 *  that performs the actual movement.
 *  This methods assumes that each movement is relative to the last stopping place.
 *  There are other ways to perform encoder based moves, but this method is probably the simplest.
 *  This code uses the RUN_TO_POSITION mode to enable the Motor controllers to generate the run profile
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@Autonomous  //(name="Pushbot: Auto Drive By Encoder", group="Pushbot")
//@Disabled
public class AutonomousJavaOpMode extends LinearOpMode {

    /* Declare OpMode members. */
    private CyberCatBot catbot;
    //private ElapsedTime     runtime = new ElapsedTime();

    // eg: HD Hex Motor https://docs.revrobotics.com/rev-control-system/sensors/encoders/motor-based-encoders
    private static final double     COUNTS_PER_MOTOR_REV    = 28 ;
    private static final double     DRIVE_GEAR_REDUCTION    = 20 ;     // 20x gear box
    private static final double     WHEEL_DIAMETER_CM   = 7.5 ;     // For figuring circumference
    private static final double     COUNTS_PER_CM         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
                                                            (WHEEL_DIAMETER_CM * 3.1415);

    private static final double AUTONOMOUS_VELOCITY = 1000;
    private static final double ZERO_VELOCITY = 0;

    @Override
    public void runOpMode() {

        /*
         * Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */

        catbot = new CyberCatBot(hardwareMap);

        telemetry.addData("Status", "Initialized");
        telemetry.update();


        // From YouTube video: https://www.youtube.com/watch?v=H3V3A7CgwPU

        // Deliver Wobble Goal to correct target zone (15 points)
             // pick the correct square to go to based on the number of rings


             // drive to the squares based on relative position
        // check all the trackable targets to see which one (if any) is visible.
        /*
        boolean targetVisible = false;
        for (VuforiaTrackable trackable : allTrackables) {
            if (((VuforiaTrackableDefaultListener)trackable.getListener()).isVisible()) {
                telemetry.addData("Visible Target", trackable.getName());
                targetVisible = true;

                // getUpdatedRobotLocation() will return null if no new information is available since
                // the last time that call was made, or if the trackable is not currently visible.
                OpenGLMatrix robotLocationTransform = ((VuforiaTrackableDefaultListener)trackable.getListener()).getUpdatedRobotLocation();
                if (robotLocationTransform != null) {
                    lastLocation = robotLocationTransform;
                }
                break;
            }
        }

        // Provide feedback as to where the robot is located (if we know).
        if (targetVisible) {
            // express position (translation) of robot in inches.
            VectorF translation = lastLocation.getTranslation();
            telemetry.addData("Pos (in)", "{X, Y, Z} = %.1f, %.1f, %.1f",
                    translation.get(0) / mmPerInch, translation.get(1) / mmPerInch, translation.get(2) / mmPerInch);

            // express the rotation of the robot in degrees.
            Orientation rotation = Orientation.getOrientation(lastLocation, EXTRINSIC, XYZ, DEGREES);
            telemetry.addData("Rot (deg)", "{Roll, Pitch, Heading} = %.0f, %.0f, %.0f", rotation.firstAngle, rotation.secondAngle, rotation.thirdAngle);
        }
        else {
            telemetry.addData("Visible Target", "none");
        }
        telemetry.update();
    }
*/
    // Disable Tracking when we are done;
        //targetsUltimateGoal.deactivate();
             // place the wobble goal completely into the square

        // Park over launch line (5 points)

        /*
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

         */

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        //go(AUTONOMOUS_VELOCITY, 40, CyberCatBot.FORWARD);
        //go(AUTONOMOUS_VELOCITY, 40, CyberCatBot.BACKWARD);
        go(AUTONOMOUS_VELOCITY, 20, CyberCatBot.RIGHT);
        go(AUTONOMOUS_VELOCITY, 20, CyberCatBot.LEFT);

        // Place ring in low goal (3 points ea.)
            // drive to goal
            // place 1 into low goal
        // Ring launched into mid goal (6 points ea.)
            // drive to behind launch line
            // shoot 2 rings into mid goal
        // Ring launched into high goal (12 points ea.)

        // Knock down Power Shot Target (15 points ea.)

        telemetry.addData("Path", "Complete");
        telemetry.update();

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
}
