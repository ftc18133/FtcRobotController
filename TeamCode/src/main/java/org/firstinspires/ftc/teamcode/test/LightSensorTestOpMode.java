package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.LightSensor;
import com.qualcomm.robotcore.hardware.ColorSensor;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp
/**
 * 3060 at full battery charge 10/11/20
 */
public class LightSensorTestOpMode extends LinearOpMode {

   //  private DistanceSensor lightSensor;
//private LightSensor lightSensor;
private ColorSensor lightSensor;
    @Override
    public void runOpMode() {
        // lightSensor = hardwareMap.get(DistanceSensor.class, "lightSensor");
        //lightSensor=hardwareMap.get(LightSensor.class,"lightSensor");
        lightSensor=hardwareMap.get(ColorSensor.class,"lightSensor");
        lightSensor.enableLed(true);
        //lightSensor.enableLed(false);
        waitForStart();


        // 2920
        while (opModeIsActive()) {
            //telemetry.addData("Distance (cm)", lightSensor.getDistance(DistanceUnit.CM));
            //telemetry.addData("Light Level",  lightSensor.getLightDetected());
            telemetry.addData("Light Level",  lightSensor.alpha());
            telemetry.update();

        }
    }
}

