package org.firstinspires.ftc.teamcode.tuning;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;

import org.apache.commons.math3.stat.inference.TTest;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.everglow_library.Utils;
import org.firstinspires.ftc.teamcode.subsystems.ArtifactColor;
import org.firstinspires.ftc.teamcode.subsystems.FeedingMechanism;

import java.util.Arrays;

@TeleOp(name="Color Sensor Testing")
@Config
public class ColorSensorTesting extends LinearOpMode {
    RevColorSensorV3 colorSensor;

    public static double purpleRedValue = 0;
    public static double purpleGreenValue = 0;
    public static double purpleBlueValue = 0;
    public static double greenRedValue = 0;
    public static double greenGreenValue = 0;
    public static double greenBlueValue = 0;
    public static int spindexerPosition = 0;

    private ArtifactColor matchColor(NormalizedRGBA rgba) {
        double[] lastColorArray = ArtifactColor.NormalizedRGBAToArray(rgba);
        if (Utils.areNormalizedRGBArraysSimilar(lastColorArray, Utils.normalizeArray(new double[]{greenRedValue, greenGreenValue, greenBlueValue}))) {
            return ArtifactColor.GREEN;
        }

        if (Utils.areNormalizedRGBArraysSimilar(lastColorArray, Utils.normalizeArray(new double[]{purpleRedValue, purpleGreenValue, purpleBlueValue}))) {
            return ArtifactColor.PURPLE;
        }

        return ArtifactColor.NONE;
    }

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        colorSensor = hardwareMap.get(RevColorSensorV3.class, "intakeSensor");
        Servo spindexerServo = hardwareMap.get(Servo.class, "spindexerServo");
        Utils.setServoPWMRange(spindexerServo, 510, 2490);

        FeedingMechanism.SpindexerPosition[] spindexerPositions = new FeedingMechanism.SpindexerPosition[3];
        spindexerPositions[0] = FeedingMechanism.SpindexerPosition.INTAKE_INDEX_0;
        spindexerPositions[1] = FeedingMechanism.SpindexerPosition.INTAKE_INDEX_1;
        spindexerPositions[2] = FeedingMechanism.SpindexerPosition.INTAKE_INDEX_2;

        waitForStart();

        while (opModeIsActive()) {
            NormalizedRGBA currentColor = colorSensor.getNormalizedColors();
            ArtifactColor matchedColor = matchColor(currentColor);

            spindexerServo.setPosition(spindexerPositions[spindexerPosition].position);

            telemetry.addData("distance", colorSensor.getDistance(DistanceUnit.MM));
            if (colorSensor.getDistance(DistanceUnit.MM) < FeedingMechanism.maxColorSensorDistance) {
                double[] rawColors = ArtifactColor.NormalizedRGBAToArray(currentColor);
                double greenDistance = Utils.getDistanceOf3dVectors(rawColors, Utils.normalizeArray(new double[]{greenRedValue, greenGreenValue, greenBlueValue}));

                double purpleDistance = Utils.getDistanceOf3dVectors(rawColors, Utils.normalizeArray(new double[]{purpleRedValue, purpleGreenValue, purpleBlueValue}));
                telemetry.addData("greenDistance", greenDistance);
                telemetry.addData("purpleDistance", purpleDistance);
                telemetry.addData("raw color value", Arrays.toString(rawColors));
                telemetry.addData("matched color", matchedColor.name());
            }
            else {
                telemetry.addLine("No artifact is in!");
            }
            telemetry.update();
        }
    }
}
