package org.firstinspires.ftc.teamcode.Tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp
public class DistanceSensorReadingTest extends OpMode {

    DistanceSensor DstSensor;

    @Override
    public void init() {

        DstSensor = hardwareMap.get(DistanceSensor.class, "rightDistanceSensor");

    }

    @Override
    public void loop() {

        double CurrDist;
        CurrDist = DstSensor.getDistance(DistanceUnit.INCH);
        telemetry.addData("CurrDist", CurrDist);
        telemetry.update();
    }
}
