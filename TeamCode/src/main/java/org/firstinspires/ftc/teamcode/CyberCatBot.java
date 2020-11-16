package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.ColorSensor;

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
    public static final double     WHEEL_DIAMETER_CM   = 7.5 ;     // For figuring circumference
    public static final double     COUNTS_PER_CM         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_CM * 3.1415);

    public static final int FORWARD = 0;
    public static final int BACKWARD = 1;
    public static final int LEFT = 2;
    public static final int RIGHT = 3;

    // PROPERTIES **********************************************************************************

    private DcMotorEx motorL1;
    private DcMotorEx motorL2;
    private DcMotorEx motorR1;
    private DcMotorEx motorR2;
    private DcMotorEx ringMotor;
    private DcMotorEx ingestMotor;
    private DcMotorEx rampMotor;
    private DcMotorEx liftMotor;
    private ColorSensor lightSensor;
    private HardwareMap hardwareMap;

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

    public DcMotorEx getRingMotor() { return ringMotor; }

    public DcMotorEx getIngestMotor() { return ingestMotor; }

    public DcMotorEx getRampMotor() { return rampMotor;}

    public DcMotorEx getLiftMotor() { return liftMotor;}

    public ColorSensor getLightSensor() { return lightSensor; }

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


        ringMotor = hardwareMap.get(DcMotorEx.class, "ringMotor");
        //ringMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        ringMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        ingestMotor = hardwareMap.get(DcMotorEx.class, "ingestMotor");
        //ingestMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        ingestMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        rampMotor = hardwareMap.get(DcMotorEx.class, "rampMotor");
        rampMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        //rampMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //rampMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        liftMotor = hardwareMap.get(DcMotorEx.class, "liftMotor");
        //ingestMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        lightSensor = hardwareMap.get(ColorSensor.class, "lightSensor");
        lightSensor.enableLed(true);
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

}
