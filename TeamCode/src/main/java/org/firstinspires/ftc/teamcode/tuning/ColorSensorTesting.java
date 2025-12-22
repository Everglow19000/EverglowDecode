package org.firstinspires.ftc.teamcode.tuning;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.everglow_library.Utils;
import org.firstinspires.ftc.teamcode.subsystems.ArtifactColor;
import org.firstinspires.ftc.teamcode.subsystems.FeedingMechanism;

import java.util.Arrays;

@TeleOp(name="Color Sensor Testing", group="Tests")
@Config
public class ColorSensorTesting extends LinearOpMode {
    RevColorSensorV3 colorSensor1;
    RevColorSensorV3 colorSensor2;

    public static double purpleRedValue = ArtifactColor.PURPLE.color[0];
    public static double purpleGreenValue = ArtifactColor.PURPLE.color[1];
    public static double purpleBlueValue = ArtifactColor.PURPLE.color[2];
    public static double greenRedValue = ArtifactColor.GREEN.color[0];
    public static double greenGreenValue = ArtifactColor.GREEN.color[1];
    public static double greenBlueValue = ArtifactColor.GREEN.color[2];
    public static int spindexerPosition = 0;
    public static double maxDistance = FeedingMechanism.artifactDistanceFromSensor;

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
        colorSensor1 = hardwareMap.get(RevColorSensorV3.class, "intakeSensor1");
        colorSensor2 = hardwareMap.get(RevColorSensorV3.class, "intakeSensor2");
        Servo spindexerServo = hardwareMap.get(Servo.class, "spindexerServo");

        FeedingMechanism.SpindexerPosition[] spindexerPositions = new FeedingMechanism.SpindexerPosition[3];
        spindexerPositions[0] = FeedingMechanism.SpindexerPosition.INTAKE_INDEX_0;
        spindexerPositions[1] = FeedingMechanism.SpindexerPosition.INTAKE_INDEX_1;
        spindexerPositions[2] = FeedingMechanism.SpindexerPosition.INTAKE_INDEX_2;

        waitForStart();

        NormalizedRGBA currentColor = null;
        ArtifactColor matchedColor = null;

        while (opModeIsActive()) {
            FeedingMechanism.artifactDistanceFromSensor = maxDistance;
            double distance1 = colorSensor1.getDistance(DistanceUnit.MM);
            double distance2 = colorSensor2.getDistance(DistanceUnit.MM);
            if (distance1 <= FeedingMechanism.artifactDistanceFromSensor || distance2 <= FeedingMechanism.artifactDistanceFromSensor) {
                if (distance1 <= FeedingMechanism.artifactDistanceFromSensor) {
                    currentColor = colorSensor1.getNormalizedColors();
                }
                else {
                    currentColor = colorSensor2.getNormalizedColors();
                }
            }
            else {
                currentColor = null;
            }

            spindexerServo.setPosition(spindexerPositions[spindexerPosition].position);

            telemetry.addData("distance1", distance1);
            telemetry.addData("distance2", distance2);
            if (distance1 <= FeedingMechanism.artifactDistanceFromSensor || distance2 <= FeedingMechanism.artifactDistanceFromSensor) {
                if (currentColor != null) {
                    matchedColor = matchColor(currentColor);
                }
                else {
                    matchedColor = null;
                }
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
