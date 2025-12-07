package org.firstinspires.ftc.teamcode.subsystems;


import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;
import org.apache.commons.math3.stat.inference.TTest;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.everglow_library.Subsystem;
import org.firstinspires.ftc.teamcode.everglow_library.Utils;
import org.opencv.core.Mat;

public class FeedingMechanism implements Subsystem {
    enum SpindexerPositions {
        SHOOT_INDEX_0(0.595),
        SHOOT_INDEX_1(1),
        SHOOT_INDEX_2(0.205),
        INTAKE_INDEX_0(0.02),
        INTAKE_INDEX_1(0.41),
        INTAKE_INDEX_2(0.79);

        public double position;

        private SpindexerPositions(double position) {
            this.position = position;
        }
    }
    // servo that pushes artifacts into the shooter
    Servo feedingServo;
    // servo that rotates the artifact in the spindexer
    Servo spindexerServo;
    ColorRangeSensor colorSensor;
    Motif motif;
    ArtifactColor[] storedArtifacts = new ArtifactColor[3];
    public FeedingMechanism(HardwareMap hardwareMap, Motif motif) {
        this.feedingServo = hardwareMap.get(Servo.class,"feedingServo");
        this.spindexerServo = hardwareMap.get(Servo.class, "spindexerServo");
        Utils.setServoPWMRange(spindexerServo, 510, 2490);
        this.colorSensor  = hardwareMap.get(ColorRangeSensor.class,"intakeSensor");
        this.motif = motif;
    }

    public void setMotif(Motif motif) {
        this.motif = motif;
    }

    private final int voidThreshold = 3;
    private int voidCount = voidThreshold+1;
    //Counts the number of subsequent times an artifact has not been detected
    //Starts one above the threshold. Once an artifact has been seen, is set to zero.
    //Only marks an artifact as entered when the count is exactly the threshold.
    //Can only mark an artifact once one has been seen
    private NormalizedRGBA lastColor;
    @Override
    public void update(int iterationCount) {

        if (colorSensor.getDistance(DistanceUnit.MM) < 10) {//Checks if an artifact is in view
            lastColor = colorSensor.getNormalizedColors();
            voidCount = 0;
        }
        else {
            voidCount++;
            if (voidCount == voidThreshold) {
                ArtifactEntered();
            }
        }
    }

    // defines artifact object using lastColor and puts it in the thing\

    private ArtifactColor MatchColor() {
        double[] lastColorArray = ArtifactColor.NormalizedRGBAToArray(lastColor);
        double greenPValue = new TTest().pairedTTest(lastColorArray,ArtifactColor.GREEN.color);
        if (greenPValue  < 0.05/2) {return ArtifactColor.GREEN;}

        double purplePValue = new TTest().pairedTTest(lastColorArray,ArtifactColor.PURPLE.color);
        if (purplePValue  < 0.05/2) {return ArtifactColor.PURPLE;}

        return ArtifactColor.NONE;
    }
    private void ArtifactEntered(){
        storedArtifacts[1] = MatchColor();
    }

    @Override
    public String status() {
        return "";
    }
}
