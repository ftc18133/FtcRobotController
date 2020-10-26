package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.LightSensor;

public class CyberCatBot {

    // CONSTANTS ***********************************************************************************

    public static final double PI_MULTIPLE = 0.1;
    public static final double F_CONSTANT = 32767;
    public static final double D_CONSTANT = 0;
    public static final double POSITION_CONSTANT = 5.0;
    public static final double WHITE_THRESHOLD = 0.2;  // spans between 0.1 - 0.5 from dark to light
    public static final double MAX_VELOCITY = 3060;

    // PROPERTIES **********************************************************************************

    private DcMotorEx motorL1;
    private DcMotorEx motorL2;
    private DcMotorEx motorR1;
    private DcMotorEx motorR2;
    private DcMotorEx ringMotor;
    private LightSensor lightSensor;
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

    public DcMotorEx getRingMotor() { return ringMotor; }

    public LightSensor getLightSensor() { return lightSensor; }


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


    private void init()
    {
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

        ringMotor = hardwareMap.get(DcMotorEx.class, "ringMotor");
        ringMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        ringMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        lightSensor = hardwareMap.get(LightSensor.class, "lightSensor");
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
