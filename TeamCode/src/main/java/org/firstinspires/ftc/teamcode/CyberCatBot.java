package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.ArrayList;
import java.util.List;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.YZX;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.BACK;

public class CyberCatBot {

    // CONSTANTS ***********************************************************************************

    public static final double PI_MULTIPLE = 0.1;
    public static final double F_CONSTANT = 32767;
    public static final double D_CONSTANT = 0;
    public static final double POSITION_CONSTANT = 5.0;
    public static final int WHITE_THRESHOLD = 500;  // spans between 0.1 - 0.5 from dark to light
    public static final double MAX_VELOCITY = 3060;

    // eg: HD Hex Motor https://docs.revrobotics.com/rev-control-system/sensors/encoders/motor-based-encoders
    public static final double     COUNTS_PER_MOTOR_REV    = 28 ;
    public static final double     DRIVE_GEAR_REDUCTION    = 20 ;     // 20x gear box
    public static final double     TOTAL_POSITION    = COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION;
    public static final double     WHEEL_DIAMETER_CM   = 7.5 ;     // For figuring circumference
    public static final double     COUNTS_PER_CM         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_CM * 3.1415);

    public static final int FORWARD = 0;
    public static final int BACKWARD = 1;
    public static final int LEFT = 2;
    public static final int RIGHT = 3;

    // WebCam variables
    // IMPORTANT: If you are using a USB WebCam, you must select CAMERA_CHOICE = BACK; and PHONE_IS_PORTRAIT = false;
    private static final VuforiaLocalizer.CameraDirection CAMERA_CHOICE = BACK;
    private static final boolean PHONE_IS_PORTRAIT = false  ;

    /*
     * IMPORTANT: You need to obtain your own license key to use Vuforia. The string below with which
     * 'parameters.vuforiaLicenseKey' is initialized is for illustration only, and will not function.
     * A Vuforia 'Development' license key, can be obtained free of charge from the Vuforia developer
     * web site at https://developer.vuforia.com/license-manager.
     *
     * Vuforia license keys are always 380 characters long, and look as if they contain mostly
     * random data.
     */
    private static final String VUFORIA_KEY =
            "AXSpSHD/////AAABmTJB0wiueUfYn5qqz/nt6vl0TURcWS0FMC0yTMvBwpeSqvJao0WyOjmhD6OveksT3/1z3983TWy/8QC2Koa/LgYGRx+0YEslttcCH1inXBshE7vIFYsKEOIo/ZFqSjUzfJrAhYkePEgPNjkyex6n3QLs0B1JT3CN05cDpmMvNARUFXzGu06W4PozYMjpQ9DgN3lxJUaZpzP4n4kOxsLC0u4KmwtaIJVorzQaKLZZaRBzMVX0u78VZ8B3gKravQBucP3R2X+hrJ5XaMh6/wDk2c2rsKl/L3yZqTX2cBI3oGMdp3W4cENuib4fudwC9XM76kEB4732+5IE9coMmgwnJrsOK/fKAtqYwfUbLunUo7jr";

    // Since ImageTarget trackables use mm to specifiy their dimensions, we must use mm for all the physical dimension.
    // We will define some constants and conversions here
    public static final float mmPerInch        = 25.4f;
    public static final float mmTargetHeight   = (6) * mmPerInch;          // the height of the center of the target image above the floor

    // Constants for perimeter targets
    private static final float halfField = 72 * mmPerInch;
    private static final float quadField  = 36 * mmPerInch;


    /**
     * This is the webcam we are to use. As with other hardware devices such as motors and
     * servos, this device is identified using the robot configuration tool in the FTC application.
     */
    WebcamName webcamName = null;
    private float phoneXRotate    = 0;
    private float phoneYRotate    = 0;
    private float phoneZRotate    = 0;

    private static final String TFOD_MODEL_ASSET = "UltimateGoal.tflite";
    public static final String LABEL_QUAD = "Quad";
    public static final String LABEL_SINGLE = "Single";

    // PROPERTIES **********************************************************************************

    private HardwareMap hardwareMap;

    private DcMotorEx motorL1;
    private DcMotorEx motorL2;
    private DcMotorEx motorR1;
    private DcMotorEx motorR2;
    private DcMotorEx ringMotor1;
    private DcMotorEx ringMotor2;
    private DcMotorEx ingestMotor;
    private CRServo rampServo;
    private DcMotorEx liftMotor;
    private ColorSensor lightSensor;
    private Servo clawServo;

    private List<VuforiaTrackable> allTrackables;
    private OpenGLMatrix lastLocation;
    private VuforiaLocalizer vuforia;
    private VuforiaTrackables targetsUltimateGoal;
    private TFObjectDetector tfod;

    // METHODS *************************************************************************************

    public CyberCatBot(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;
        init();
    }

    public DcMotorEx getMotorL1()
    {
        return motorL1;
    }

    public DcMotorEx getMotorL2()
    {
        return motorL2;
    }

    public DcMotorEx getMotorR1()
    {
        return motorR1;
    }

    public DcMotorEx getMotorR2()
    {
        return motorR2;
    }

    public void setMotorMode(DcMotor.RunMode runMode)
    {
        getMotorL1().setMode(runMode);
        getMotorL2().setMode(runMode);
        getMotorR1().setMode(runMode);
        getMotorR2().setMode(runMode);
    }

    public DcMotorEx getRingMotor1() { return ringMotor1; }

    public DcMotorEx getRingMotor2() { return ringMotor2; }

    public DcMotorEx getIngestMotor() { return ingestMotor; }

    public CRServo getRampServo() { return rampServo;}

    public DcMotorEx getLiftMotor() { return liftMotor;}

    public ColorSensor getLightSensor() { return lightSensor; }

    public Servo getClawServo() { return clawServo; }

    public VuforiaTrackables getTargetsUltimateGoal() { return targetsUltimateGoal; }

    public List<VuforiaTrackable> getAllTrackables() { return allTrackables; }

    public OpenGLMatrix getLastLocation() { return lastLocation; }

    public void setLastLocation(OpenGLMatrix lastLocation) { this.lastLocation = lastLocation; }

    public TFObjectDetector getTfod() { return tfod; }

    public void setVelocity(double velocity)
    {
        motorL1.setVelocity(velocity);
        motorL2.setVelocity(velocity);
        motorR1.setVelocity(velocity);
        motorR2.setVelocity(velocity);
    }

    public void setVelocity(double l1Velocity, double l2Velocity, double r1Velocity, double r2Velocity)
    {
        motorL1.setVelocity(l1Velocity);
        motorL2.setVelocity(l2Velocity);
        motorR1.setVelocity(r1Velocity);
        motorR2.setVelocity(r2Velocity);
    }

    public void strafeVelocity(double velocity)
    {
        motorL1.setVelocity(velocity);
        motorL2.setVelocity(-velocity);
        motorR1.setVelocity(-velocity);
        motorR2.setVelocity(velocity);
    }

    private void init()
    {
        motorL1 = hardwareMap.get(DcMotorEx.class, "motorL1");
        motorL1.setDirection(DcMotorSimple.Direction.REVERSE);
        initMotor(motorL1);

        motorL2 = hardwareMap.get(DcMotorEx.class, "motorL2");
        motorL2.setDirection(DcMotorSimple.Direction.REVERSE);
        initMotor(motorL2);

        motorR1 = hardwareMap.get(DcMotorEx.class, "motorR1");
        //motorR1.setDirection(DcMotorSimple.Direction.REVERSE);
        initMotor(motorR1);

        motorR2 = hardwareMap.get(DcMotorEx.class, "motorR2");
        //motorR2.setDirection(DcMotorSimple.Direction.REVERSE);
        initMotor(motorR2);


        ringMotor1 = hardwareMap.get(DcMotorEx.class, "ringMotor1");
        ringMotor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        ringMotor2 = hardwareMap.get(DcMotorEx.class, "ringMotor2");
        ringMotor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        ringMotor2.setDirection(DcMotorSimple.Direction.REVERSE);

        ingestMotor = hardwareMap.get(DcMotorEx.class, "ingestMotor");
        //ingestMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        ingestMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //rampMotor = hardwareMap.get(DcMotorEx.class, "rampMotor");
        rampServo = hardwareMap.get(CRServo.class, "rampServo");
        //rampMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        //rampMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //rampMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        liftMotor = hardwareMap.get(DcMotorEx.class, "liftMotor");
        //liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //liftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        lightSensor = hardwareMap.get(ColorSensor.class, "lightSensor");
        lightSensor.enableLed(true);

        clawServo = hardwareMap.get(Servo.class, "clawServo");

        initWebcam();
        initTensorFlow();
    }

    private void initMotor(DcMotorEx motor)
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
        double fValue = F_CONSTANT / MAX_VELOCITY;
        double pValue = PI_MULTIPLE * fValue;
        double iValue = PI_MULTIPLE * pValue;

        motor.setVelocityPIDFCoefficients(pValue, iValue, D_CONSTANT, fValue);
        motor.setPositionPIDFCoefficients(POSITION_CONSTANT);
    }
    private void initWebcam()
    {
        /*
         * Retrieve the camera we are to use.
         */
        webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");

        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         * We can pass Vuforia the handle to a camera preview resource (on the RC phone);
         * If no camera monitor is desired, use the parameter-less constructor instead (commented out below).
         */
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        // VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;

        /**
         * We also indicate which camera on the RC we wish to use.
         */
        parameters.cameraName = webcamName;

        // Make sure extended tracking is disabled for this example.
        parameters.useExtendedTracking = false;

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Load the data sets for the trackable objects. These particular data
        // sets are stored in the 'assets' part of our application.
        this.targetsUltimateGoal = this.vuforia.loadTrackablesFromAsset("UltimateGoal");
        VuforiaTrackable blueTowerGoalTarget = targetsUltimateGoal.get(0);
        blueTowerGoalTarget.setName("Blue Tower Goal Target");
        VuforiaTrackable redTowerGoalTarget = targetsUltimateGoal.get(1);
        redTowerGoalTarget.setName("Red Tower Goal Target");
        VuforiaTrackable redAllianceTarget = targetsUltimateGoal.get(2);
        redAllianceTarget.setName("Red Alliance Target");
        VuforiaTrackable blueAllianceTarget = targetsUltimateGoal.get(3);
        blueAllianceTarget.setName("Blue Alliance Target");
        VuforiaTrackable frontWallTarget = targetsUltimateGoal.get(4);
        frontWallTarget.setName("Front Wall Target");

        // For convenience, gather together all the trackable objects in one easily-iterable collection */
        allTrackables = new ArrayList<VuforiaTrackable>();
        allTrackables.addAll(targetsUltimateGoal);

        /**
         * In order for localization to work, we need to tell the system where each target is on the field, and
         * where the phone resides on the robot.  These specifications are in the form of <em>transformation matrices.</em>
         * Transformation matrices are a central, important concept in the math here involved in localization.
         * See <a href="https://en.wikipedia.org/wiki/Transformation_matrix">Transformation Matrix</a>
         * for detailed information. Commonly, you'll encounter transformation matrices as instances
         * of the {@link OpenGLMatrix} class.
         *
         * If you are standing in the Red Alliance Station looking towards the center of the field,
         *     - The X axis runs from your left to the right. (positive from the center to the right)
         *     - The Y axis runs from the Red Alliance Station towards the other side of the field
         *       where the Blue Alliance Station is. (Positive is from the center, towards the BlueAlliance station)
         *     - The Z axis runs from the floor, upwards towards the ceiling.  (Positive is above the floor)
         *
         * Before being transformed, each target image is conceptually located at the origin of the field's
         *  coordinate system (the center of the field), facing up.
         */

        //Set the position of the perimeter targets with relation to origin (center of field)
        redAllianceTarget.setLocation(OpenGLMatrix
                .translation(0, -halfField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 180)));

        blueAllianceTarget.setLocation(OpenGLMatrix
                .translation(0, halfField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 0)));
        frontWallTarget.setLocation(OpenGLMatrix
                .translation(-halfField, 0, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0 , 90)));

        // The tower goal targets are located a quarter field length from the ends of the back perimeter wall.
        blueTowerGoalTarget.setLocation(OpenGLMatrix
                .translation(halfField, quadField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0 , -90)));
        redTowerGoalTarget.setLocation(OpenGLMatrix
                .translation(halfField, -quadField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, -90)));

        //
        // Create a transformation matrix describing where the phone is on the robot.
        //
        // NOTE !!!!  It's very important that you turn OFF your phone's Auto-Screen-Rotation option.
        // Lock it into Portrait for these numbers to work.
        //
        // Info:  The coordinate frame for the robot looks the same as the field.
        // The robot's "forward" direction is facing out along X axis, with the LEFT side facing out along the Y axis.
        // Z is UP on the robot.  This equates to a bearing angle of Zero degrees.
        //
        // The phone starts out lying flat, with the screen facing Up and with the physical top of the phone
        // pointing to the LEFT side of the Robot.
        // The two examples below assume that the camera is facing forward out the front of the robot.

        // We need to rotate the camera around it's long axis to bring the correct camera forward.
        if (CAMERA_CHOICE == BACK) {
            phoneYRotate = -90;
        } else {
            phoneYRotate = 90;
        }

        // Rotate the phone vertical about the X axis if it's in portrait mode
        if (PHONE_IS_PORTRAIT) {
            phoneXRotate = 90 ;
        }

        // Next, translate the camera lens to where it is on the robot.
        // In this example, it is centered (left to right), but forward of the middle of the robot, and above ground level.
        final float CAMERA_FORWARD_DISPLACEMENT  = 9.0f * mmPerInch;   // eg: Camera is 4 Inches in front of robot-center (Might need to adjust)
        final float CAMERA_VERTICAL_DISPLACEMENT = 5.0f * mmPerInch;   // eg: Camera is 8 Inches above ground
        final float CAMERA_LEFT_DISPLACEMENT     = -2.0f * mmPerInch;     // eg: Camera is ON the robot's center line

        OpenGLMatrix robotFromCamera = OpenGLMatrix
                .translation(CAMERA_FORWARD_DISPLACEMENT, CAMERA_LEFT_DISPLACEMENT, CAMERA_VERTICAL_DISPLACEMENT)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, YZX, DEGREES, phoneYRotate, phoneZRotate, phoneXRotate));

        /**  Let all the trackable listeners know where the phone is.  */
        for (VuforiaTrackable trackable : allTrackables) {
            ((VuforiaTrackableDefaultListener) trackable.getListener()).setPhoneInformation(robotFromCamera, parameters.cameraDirection);
        }

    }

    private void initTensorFlow() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 0.8f;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_QUAD, LABEL_SINGLE);
    }

    public void liftArm(){
        liftMotor.setPower(0.2);
        //liftMotor.setTargetPosition(0);
        //liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    public void lowerArm(){
        liftMotor.setPower(-0.2);
        //liftMotor.setTargetPosition((int)(TOTAL_POSITION / 4));
        //liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    public void stopArm () {
        getLiftMotor().setPower(0);
    }
}
