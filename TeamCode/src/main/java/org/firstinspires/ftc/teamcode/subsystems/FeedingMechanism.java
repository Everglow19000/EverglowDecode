package org.firstinspires.ftc.teamcode.subsystems;


import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;
import org.apache.commons.math3.stat.inference.TTest;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.everglow_library.Subsystem;
import org.firstinspires.ftc.teamcode.everglow_library.Utils;

public class FeedingMechanism implements Subsystem {
    public enum FeedingServoPosition {
        DOWN(0.5),
        UP(0.7);

        public double position;

        private FeedingServoPosition(double position) {
            this.position = position;
        }
    }

    public enum SpindexerPosition {
        SHOOT_INDEX_0(0.595),
        SHOOT_INDEX_1(0.985),
        SHOOT_INDEX_2(0.205),
        INTAKE_INDEX_0(0.02),
        INTAKE_INDEX_1(0.41),
        INTAKE_INDEX_2(0.79);

        public double position;

        private SpindexerPosition(double position) {
            this.position = position;
        }

        public SpindexerPosition getNext() {
            switch (this) {
                case SHOOT_INDEX_0:
                    return SHOOT_INDEX_1;
                case SHOOT_INDEX_1:
                    return SHOOT_INDEX_2;
                case SHOOT_INDEX_2:
                    return SHOOT_INDEX_0;
                case INTAKE_INDEX_0:
                    return INTAKE_INDEX_1;
                case INTAKE_INDEX_1:
                    return INTAKE_INDEX_2;
                case INTAKE_INDEX_2:
                    return INTAKE_INDEX_0;
                default:
                    return INTAKE_INDEX_0;
            }
        }

        public int getSelectedArrayIndex() {
            if (this == SHOOT_INDEX_0 || this == INTAKE_INDEX_0) {
                return 0;
            }
            if (this == SHOOT_INDEX_1 || this == INTAKE_INDEX_1) {
                return 1;
            }
            if (this == SHOOT_INDEX_2 || this == INTAKE_INDEX_2) {
                return 2;
            }
            return 0;
        }

        public static SpindexerPosition shootIndex(int index) {
            if (index == 0) {
                return SHOOT_INDEX_0;
            }
            if (index == 1) {
                return SHOOT_INDEX_1;
            }
            if (index == 2) {
                return SHOOT_INDEX_2;
            }
            return SHOOT_INDEX_0;
        }
    }

    // moves the spindexer to the next position, and raises and lowers the feeding servo. assumes shooter is already active and at speed
    public class FeedSingleArtifactAction implements Action {
        SpindexerPosition position;
        boolean hasStartedFeeding = false;
        boolean hasStartedReturn = false;

        // both in ms
        double feedingStartTime;
        static final double moveTime = 500;
        public FeedSingleArtifactAction(SpindexerPosition position) {
            this.position = position;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            if (!hasStartedFeeding) {
                hasStartedFeeding = true;
                feedingStartTime = System.currentTimeMillis();
                setFeedingServoPosition(FeedingServoPosition.UP);
            }

            if (!limitSwitch.getState() && hasStartedFeeding && !hasStartedReturn && System.currentTimeMillis() - feedingStartTime > moveTime) {
                hasStartedReturn = true;
                setFeedingServoPosition(FeedingServoPosition.DOWN);
            }

            return hasStartedFeeding && System.currentTimeMillis() - feedingStartTime > moveTime && !limitSwitch.getState();
        }
    }

    // servo that pushes artifacts into the shooter
    Servo feedingServo;
    // servo that rotates the artifact in the spindexer
    Servo spindexerServo;
    ColorRangeSensor colorSensor;
    //swyft sense magnetic limit switch, true if feedingServo is down
    DigitalChannel limitSwitch;
    SpindexerPosition currentSpindexerPosition;
    FeedingServoPosition currentFeedingServoPosition;
    Motif motif;
    ArtifactColor[] storedArtifacts = new ArtifactColor[3];
    public FeedingMechanism(HardwareMap hardwareMap, Motif motif) {
        this.feedingServo = hardwareMap.get(Servo.class,"feedingServo");
        feedingServo.setDirection(Servo.Direction.REVERSE);
        this.spindexerServo = hardwareMap.get(Servo.class, "spindexerServo");
        Utils.setServoPWMRange(spindexerServo, 510, 2490);
        this.colorSensor  = hardwareMap.get(ColorRangeSensor.class,"intakeSensor");
        this.limitSwitch = hardwareMap.get(DigitalChannel.class, "feedingLimitSwitch");
        this.limitSwitch.setMode(DigitalChannel.Mode.INPUT);
        this.motif = motif;
        currentSpindexerPosition = SpindexerPosition.INTAKE_INDEX_0;
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
                artifactEntered();
            }
        }
    }

    // defines artifact object using lastColor and puts it in the thing\

    private ArtifactColor matchColor() {
        double[] lastColorArray = ArtifactColor.NormalizedRGBAToArray(lastColor);
        double greenPValue = new TTest().pairedTTest(lastColorArray,ArtifactColor.GREEN.color);
        if (greenPValue  < 0.05/2) {return ArtifactColor.GREEN;}

        double purplePValue = new TTest().pairedTTest(lastColorArray,ArtifactColor.PURPLE.color);
        if (purplePValue  < 0.05/2) {return ArtifactColor.PURPLE;}

        return ArtifactColor.NONE;
    }
    private void artifactEntered(){
        storedArtifacts[currentSpindexerPosition.getSelectedArrayIndex()] = matchColor();
        if (currentSpindexerPosition != SpindexerPosition.INTAKE_INDEX_2) {
            setSpindexerPosition(currentSpindexerPosition.getNext());
        }
    }
    private void setSpindexerPosition(SpindexerPosition position) {
        currentSpindexerPosition = position;
        spindexerServo.setPosition(position.position);
    }
    private void setFeedingServoPosition(FeedingServoPosition position) {
        currentFeedingServoPosition = position;
        feedingServo.setPosition(position.position);
    }

    private int countArtifactsInSpindexer() {
        int sum = 0;
        for (int i = 0; i < storedArtifacts.length; i++) {
            if (storedArtifacts[i] != ArtifactColor.NONE) {
                sum++;
            }
        }
        return sum;
    }

    private int getArtifactColorCount(ArtifactColor color) {
        int sum = 0;
        for (int i = 0; i < storedArtifacts.length; i++) {
            if (storedArtifacts[i] == color) {
                sum++;
            }
        }
        return sum;
    }

    private int[] getArtifactColorPositions(ArtifactColor color) {
        int[] positions = new int[getArtifactColorCount(color)];
        int positionsIndex = 0;
        for (int i = 0; i < storedArtifacts.length; i++) {
            if (storedArtifacts[i] == color) {
                positions[positionsIndex] = i;
                positionsIndex++;
            }
        }
        return positions;
    }

    public SpindexerPosition[] getShootingSequence() {
        SpindexerPosition[] result = new SpindexerPosition[countArtifactsInSpindexer(storedArtifacts)];

        if (motif == Motif.NONE) {
            SpindexerPosition currIndexStored;
            if (currentSpindexerPosition == SpindexerPosition.SHOOT_INDEX_0 || currentSpindexerPosition == SpindexerPosition.SHOOT_INDEX_1 || currentSpindexerPosition == SpindexerPosition.SHOOT_INDEX_2) {
                currIndexStored = currentSpindexerPosition;
            }
            else {
                currIndexStored = SpindexerPosition.SHOOT_INDEX_0;
            }
            for (int i = 0; i < result.length; i++) {
                while (storedArtifacts[currIndexStored.getSelectedArrayIndex()] == ArtifactColor.NONE) {
                    currIndexStored = currIndexStored.getNext();
                }
                result[i] = currIndexStored;
                currIndexStored = currIndexStored.getNext();
            }
        }
        else {
            int[] purpleArtifactPositions = getArtifactColorPositions(storedArtifacts, ArtifactColor.PURPLE);
            int[] greenArtifactPositions = getArtifactColorPositions(storedArtifacts, ArtifactColor.GREEN);
            if (motif == Motif.GPP) {
                int purpleIndex = 0;
                int greenIndex = 0;
                ArtifactColor preferredNow = ArtifactColor.GREEN;
                for (int i = 0; i < result.length; i++) {
                    if (preferredNow == ArtifactColor.GREEN && greenIndex >= greenArtifactPositions.length) {
                        preferredNow = ArtifactColor.PURPLE;
                    }
                    else if (preferredNow == ArtifactColor.PURPLE && purpleIndex >= purpleArtifactPositions.length) {
                        preferredNow = ArtifactColor.GREEN;
                    }

                    if (preferredNow == ArtifactColor.GREEN) {
                        result[i] = SpindexerPosition.shootIndex(greenArtifactPositions[greenIndex]);
                        greenIndex++;
                        if (greenIndex < greenArtifactPositions.length) {
                            preferredNow = ArtifactColor.GREEN;
                        }
                        else {
                            preferredNow = ArtifactColor.PURPLE;
                        }
                    }
                    else if (preferredNow == ArtifactColor.PURPLE && purpleIndex < purpleArtifactPositions.length) {
                        result[i] = SpindexerPosition.shootIndex(purpleArtifactPositions[purpleIndex]);
                        purpleIndex++;
                        if (purpleIndex < purpleArtifactPositions.length) {
                            preferredNow = ArtifactColor.PURPLE;
                        }
                        else {
                            preferredNow = ArtifactColor.GREEN;
                        }
                    }
                }
            }
            else if (motif == Motif.PGP) {
                int purpleIndex = 0;
                int greenIndex = 0;
                ArtifactColor preferredNow = ArtifactColor.PURPLE;
                for (int i = 0; i < result.length; i++) {
                    if (preferredNow == ArtifactColor.GREEN && greenIndex >= greenArtifactPositions.length) {
                        preferredNow = ArtifactColor.PURPLE;
                    }
                    else if (preferredNow == ArtifactColor.PURPLE && purpleIndex >= purpleArtifactPositions.length) {
                        preferredNow = ArtifactColor.GREEN;
                    }

                    if (preferredNow == ArtifactColor.GREEN) {
                        result[i] = SpindexerPosition.shootIndex(greenArtifactPositions[greenIndex]);
                        greenIndex++;
                        if (greenIndex < greenArtifactPositions.length) {
                            preferredNow = ArtifactColor.GREEN;
                        }
                        else {
                            preferredNow = ArtifactColor.PURPLE;
                        }
                    }
                    else if (preferredNow == ArtifactColor.PURPLE && purpleIndex < purpleArtifactPositions.length) {
                        result[i] = SpindexerPosition.shootIndex(purpleArtifactPositions[purpleIndex]);
                        purpleIndex++;
                        if (purpleIndex < purpleArtifactPositions.length) {
                            preferredNow = ArtifactColor.GREEN;
                        }
                        else {
                            preferredNow = ArtifactColor.PURPLE;
                        }
                    }
                }
            }
            else if (motif == Motif.PPG) {
                int purpleIndex = 0;
                int greenIndex = 0;
                ArtifactColor preferredNow = ArtifactColor.PURPLE;
                for (int i = 0; i < result.length; i++) {
                    if (preferredNow == ArtifactColor.GREEN && greenIndex >= greenArtifactPositions.length) {
                        preferredNow = ArtifactColor.PURPLE;
                    }
                    else if (preferredNow == ArtifactColor.PURPLE && purpleIndex >= purpleArtifactPositions.length) {
                        preferredNow = ArtifactColor.GREEN;
                    }

                    if (preferredNow == ArtifactColor.GREEN) {
                        result[i] = SpindexerPosition.shootIndex(greenArtifactPositions[greenIndex]);
                        greenIndex++;
                        if (greenIndex < greenArtifactPositions.length) {
                            preferredNow = ArtifactColor.GREEN;
                        }
                        else {
                            preferredNow = ArtifactColor.PURPLE;
                        }
                    }
                    else if (preferredNow == ArtifactColor.PURPLE && purpleIndex < purpleArtifactPositions.length) {
                        result[i] = SpindexerPosition.shootIndex(purpleArtifactPositions[purpleIndex]);
                        purpleIndex++;
                        if (purpleIndex < purpleArtifactPositions.length) {
                            preferredNow = ArtifactColor.PURPLE;
                        }
                        else {
                            preferredNow = ArtifactColor.GREEN;
                        }
                    }
                }
            }
        }

        return result;
    }

    // TODO: ALL THOSE BELOW THIS COMMENT ARE TESTS AND SHOULD NOT BE USED

    @Deprecated
    public static int countArtifactsInSpindexer(ArtifactColor[] storedArtifacts) {
        int sum = 0;
        for (int i = 0; i < storedArtifacts.length; i++) {
            if (storedArtifacts[i] != ArtifactColor.NONE) {
                sum++;
            }
        }
        return sum;
    }

    @Deprecated
    public static int getArtifactColorCount(ArtifactColor[] storedArtifacts, ArtifactColor color) {
        int sum = 0;
        for (int i = 0; i < storedArtifacts.length; i++) {
            if (storedArtifacts[i] == color) {
                sum++;
            }
        }
        return sum;
    }

    @Deprecated
    public static int[] getArtifactColorPositions(ArtifactColor[] storedArtifacts, ArtifactColor color) {
        int[] positions = new int[getArtifactColorCount(storedArtifacts, color)];
        int positionsIndex = 0;
        for (int i = 0; i < storedArtifacts.length; i++) {
            if (storedArtifacts[i] == color) {
                positions[positionsIndex] = i;
                positionsIndex++;
            }
        }
        return positions;
    }
    @Deprecated
    public static SpindexerPosition[] getShootingSequence(Motif motif, ArtifactColor[] storedArtifacts, SpindexerPosition currentSpindexerPosition) {
        SpindexerPosition[] result = new SpindexerPosition[countArtifactsInSpindexer(storedArtifacts)];

        if (motif == Motif.NONE) {
            SpindexerPosition currIndexStored;
            if (currentSpindexerPosition == SpindexerPosition.SHOOT_INDEX_0 || currentSpindexerPosition == SpindexerPosition.SHOOT_INDEX_1 || currentSpindexerPosition == SpindexerPosition.SHOOT_INDEX_2) {
                currIndexStored = currentSpindexerPosition;
            }
            else {
                currIndexStored = SpindexerPosition.SHOOT_INDEX_0;
            }
            for (int i = 0; i < result.length; i++) {
                while (storedArtifacts[currIndexStored.getSelectedArrayIndex()] == ArtifactColor.NONE) {
                    currIndexStored = currIndexStored.getNext();
                }
                result[i] = currIndexStored;
                currIndexStored = currIndexStored.getNext();
            }
        }
        else {
            int[] purpleArtifactPositions = getArtifactColorPositions(storedArtifacts, ArtifactColor.PURPLE);
            int[] greenArtifactPositions = getArtifactColorPositions(storedArtifacts, ArtifactColor.GREEN);
            if (motif == Motif.GPP) {
                int purpleIndex = 0;
                int greenIndex = 0;
                ArtifactColor preferredNow = ArtifactColor.GREEN;
                for (int i = 0; i < result.length; i++) {
                    if (preferredNow == ArtifactColor.GREEN && greenIndex >= greenArtifactPositions.length) {
                        preferredNow = ArtifactColor.PURPLE;
                    }
                    else if (preferredNow == ArtifactColor.PURPLE && purpleIndex >= purpleArtifactPositions.length) {
                        preferredNow = ArtifactColor.GREEN;
                    }

                    if (preferredNow == ArtifactColor.GREEN) {
                        result[i] = SpindexerPosition.shootIndex(greenArtifactPositions[greenIndex]);
                        greenIndex++;
                        if (greenIndex < greenArtifactPositions.length) {
                            preferredNow = ArtifactColor.GREEN;
                        }
                        else {
                            preferredNow = ArtifactColor.PURPLE;
                        }
                    }
                    else if (preferredNow == ArtifactColor.PURPLE && purpleIndex < purpleArtifactPositions.length) {
                        result[i] = SpindexerPosition.shootIndex(purpleArtifactPositions[purpleIndex]);
                        purpleIndex++;
                        if (purpleIndex < purpleArtifactPositions.length) {
                            preferredNow = ArtifactColor.PURPLE;
                        }
                        else {
                            preferredNow = ArtifactColor.GREEN;
                        }
                    }
                }
            }
            else if (motif == Motif.PGP) {
                int purpleIndex = 0;
                int greenIndex = 0;
                ArtifactColor preferredNow = ArtifactColor.PURPLE;
                for (int i = 0; i < result.length; i++) {
                    if (preferredNow == ArtifactColor.GREEN && greenIndex >= greenArtifactPositions.length) {
                        preferredNow = ArtifactColor.PURPLE;
                    }
                    else if (preferredNow == ArtifactColor.PURPLE && purpleIndex >= purpleArtifactPositions.length) {
                        preferredNow = ArtifactColor.GREEN;
                    }

                    if (preferredNow == ArtifactColor.GREEN) {
                        result[i] = SpindexerPosition.shootIndex(greenArtifactPositions[greenIndex]);
                        greenIndex++;
                        if (greenIndex < greenArtifactPositions.length) {
                            preferredNow = ArtifactColor.GREEN;
                        }
                        else {
                            preferredNow = ArtifactColor.PURPLE;
                        }
                    }
                    else if (preferredNow == ArtifactColor.PURPLE && purpleIndex < purpleArtifactPositions.length) {
                        result[i] = SpindexerPosition.shootIndex(purpleArtifactPositions[purpleIndex]);
                        purpleIndex++;
                        if (purpleIndex < purpleArtifactPositions.length) {
                            preferredNow = ArtifactColor.GREEN;
                        }
                        else {
                            preferredNow = ArtifactColor.PURPLE;
                        }
                    }
                }
            }
            else if (motif == Motif.PPG) {
                int purpleIndex = 0;
                int greenIndex = 0;
                ArtifactColor preferredNow = ArtifactColor.PURPLE;
                for (int i = 0; i < result.length; i++) {
                    if (preferredNow == ArtifactColor.GREEN && greenIndex >= greenArtifactPositions.length) {
                        preferredNow = ArtifactColor.PURPLE;
                    }
                    else if (preferredNow == ArtifactColor.PURPLE && purpleIndex >= purpleArtifactPositions.length) {
                        preferredNow = ArtifactColor.GREEN;
                    }

                    if (preferredNow == ArtifactColor.GREEN) {
                        result[i] = SpindexerPosition.shootIndex(greenArtifactPositions[greenIndex]);
                        greenIndex++;
                        if (greenIndex < greenArtifactPositions.length) {
                            preferredNow = ArtifactColor.GREEN;
                        }
                        else {
                            preferredNow = ArtifactColor.PURPLE;
                        }
                    }
                    else if (preferredNow == ArtifactColor.PURPLE && purpleIndex < purpleArtifactPositions.length) {
                        result[i] = SpindexerPosition.shootIndex(purpleArtifactPositions[purpleIndex]);
                        purpleIndex++;
                        if (purpleIndex < purpleArtifactPositions.length) {
                            preferredNow = ArtifactColor.PURPLE;
                        }
                        else {
                            preferredNow = ArtifactColor.GREEN;
                        }
                    }
                }
            }
        }

        return result;
    }

    @Override
    public String status() {
        return "";
    }
}
