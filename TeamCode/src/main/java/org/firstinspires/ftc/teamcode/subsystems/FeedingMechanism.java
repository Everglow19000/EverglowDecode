package org.firstinspires.ftc.teamcode.subsystems;


import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.SequentialAction;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.everglow_library.Subsystem;
import org.firstinspires.ftc.teamcode.everglow_library.Utils;

public class FeedingMechanism implements Subsystem {
    public static double artifactDistanceFromSensor = 39;
    public static double restOfRobotDistanceFromSensor = 42;
    public static double analogInputMin = 0.05484;
    public static double analogInputMax = 0.94939;
    public static double spindexerEncoderTolerance = 0.005;
    public enum FeedingServoPosition {
        DOWN(0.5),
        UP(0.74);

        public double position;

        private FeedingServoPosition(double position) {
            this.position = position;
        }

        public FeedingServoPosition toggle() {
            if (this == DOWN) {
                return UP;
            }
            else {
                return DOWN;
            }
        }
    }

    public enum SpindexerPosition {
        SHOOT_INDEX_0(0.75),
        SHOOT_INDEX_1(0),
        SHOOT_INDEX_2(0.37),
        INTAKE_INDEX_0(0.19),
        INTAKE_INDEX_1(0.56),
        INTAKE_INDEX_2(0.92);

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

        public SpindexerPosition getPrevious() {
            switch (this) {
                case SHOOT_INDEX_0:
                    return SHOOT_INDEX_2;
                case SHOOT_INDEX_1:
                    return SHOOT_INDEX_0;
                case SHOOT_INDEX_2:
                    return SHOOT_INDEX_1;
                case INTAKE_INDEX_0:
                    return INTAKE_INDEX_2;
                case INTAKE_INDEX_1:
                    return INTAKE_INDEX_0;
                case INTAKE_INDEX_2:
                    return INTAKE_INDEX_1;
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
        static final double moveTime = 1000;
        double returnStartTime;
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
            if (hasStartedFeeding && !hasStartedReturn && System.currentTimeMillis() - feedingStartTime >= moveTime) {
                hasStartedReturn = true;
                returnStartTime = System.currentTimeMillis();
                setFeedingServoPosition(FeedingServoPosition.DOWN);
                storedArtifacts[position.getSelectedArrayIndex()] = null;
            }

            if (hasStartedReturn && limitSwitch.getState() && System.currentTimeMillis() - returnStartTime >= moveTime) {
                setFeedingServoPosition(FeedingServoPosition.UP);
                setSpindexerPosition(position);
                hasStartedReturn = false;
            }

            return !(hasStartedReturn && !limitSwitch.getState());
        }
    }


    public class ScanCurrentArtifactAction implements Action {
        private double noneSeenCount = 0;
        private final double noneSeenThreshold = 30;
        private ScanCurrentArtifactAction() {

        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            if (isThereArtifactInCurrentIntake(true)) {
                storedArtifacts[targetSpindexerPosition.getSelectedArrayIndex()] = lastArtifactColor;
                return false;
            }
            else if (noneSeenCount < noneSeenThreshold) {
                noneSeenCount++;
            }
            else {
                storedArtifacts[targetSpindexerPosition.getSelectedArrayIndex()] = null;
                return false;
            }
            return true;
        }
    }

    public class MoveSpindexerAction implements Action {
        private SpindexerPosition targetPosition;
        private boolean hasStarted = false;
        private MoveSpindexerAction(SpindexerPosition targetPosition) {
            this.targetPosition = targetPosition;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            if (!hasStarted) {
                hasStarted = true;
                setSpindexerPosition(targetPosition);
            }
            return !isSpindexerInPosition();
        }
    }

    // servo that pushes artifacts into the shooter
    public Servo feedingServo;
    // servo that rotates the artifact in the spindexer
    public Servo spindexerServo;
    // analog input from the spindexer axon servo
    public AnalogInput spindexerEncoder;
    RevColorSensorV3 colorSensor1;
    RevColorSensorV3 colorSensor2;
    //swyft sense magnetic limit switch, true if feedingServo is down
    public DigitalChannel limitSwitch;
    SpindexerPosition targetSpindexerPosition;
    FeedingServoPosition currentFeedingServoPosition;
    Motif motif;
    ArtifactColor[] storedArtifacts = new ArtifactColor[3];
    public FeedingMechanism(HardwareMap hardwareMap, Motif motif) {
        this.feedingServo = hardwareMap.get(Servo.class,"feedingServo");
        feedingServo.setDirection(Servo.Direction.REVERSE);
        spindexerServo = hardwareMap.get(Servo.class, "spindexerServo");
        this.colorSensor1 = hardwareMap.get(RevColorSensorV3.class,"intakeSensor1");
        this.colorSensor2 = hardwareMap.get(RevColorSensorV3.class,"intakeSensor2");
        this.limitSwitch = hardwareMap.get(DigitalChannel.class, "feedingLimitSwitch");
        this.limitSwitch.setMode(DigitalChannel.Mode.INPUT);
        this.motif = motif;
        spindexerEncoder = hardwareMap.get(AnalogInput.class, "spindexerInput");
        SpindexerPosition[] values = SpindexerPosition.values();
        int minIndex = 0;

        for (int i = 0; i < values.length; i++) {
            if (Math.abs(getScaledSpindexerEncoderPosition() - values[i].position) < Math.abs(getScaledSpindexerEncoderPosition() - values[minIndex].position)) {
                minIndex = i;
            }
        }
        targetSpindexerPosition = values[minIndex];
    }

    public void setMotif(Motif motif) {
        this.motif = motif;
    }

    private final int voidThreshold = 10;
    // represents how many times we haven't been able to match a color
    private int voidCount = 0;
    //Counts the number of subsequent times an artifact has not been detected
    //Starts one above the threshold. Once an artifact has been seen, is set to zero.
    //Only marks an artifact as entered when the count is exactly the threshold.
    //Can only mark an artifact once one has been seen
    private NormalizedRGBA lastColor;
    private ArtifactColor lastArtifactColor;

    private boolean isIntaking = false;
    private boolean isIntakingLastUpdate = false;
    private boolean readyForNext = true;
    @Override
    public void update(int iterationCount) {
        if (!readyForNext) {
            readyForNext = isSpindexerInPosition();
        }
        else if (isIntaking && isThereArtifactInCurrentIntake(true)) {
            readyForNext = false;
            if (!artifactEntered(lastArtifactColor)) {
                isIntakingLastUpdate = isIntaking;
                stopIntaking();
                return;
            }
        }
        isIntakingLastUpdate = isIntaking;
    }

    public boolean isThereArtifactInCurrentIntake(boolean updateLastColor) {
        double sensor1Distance = colorSensor1.getDistance(DistanceUnit.MM);
        double sensor2Distance = colorSensor2.getDistance(DistanceUnit.MM);

        if (!updateLastColor) {
            return sensor1Distance <= artifactDistanceFromSensor || sensor2Distance <= artifactDistanceFromSensor;
        }

        if (!(sensor1Distance <= artifactDistanceFromSensor || sensor2Distance <= artifactDistanceFromSensor)) {
            lastColor = null;
            lastArtifactColor = null;
            return false;
        }

        NormalizedRGBA color1Value = colorSensor1.getNormalizedColors();
        NormalizedRGBA color2Value = colorSensor2.getNormalizedColors();

        ArtifactColor color1 = null;
        ArtifactColor color2 = null;

        if (sensor1Distance <= artifactDistanceFromSensor) {
            color1 = matchColor(color1Value);
        }
        if (sensor2Distance <= artifactDistanceFromSensor) {
            color2 = matchColor(color2Value);
        }

        if (color1 != null) {
            if (color1 != ArtifactColor.NONE) {
                lastColor = color1Value;
                lastArtifactColor = color1;
                voidCount = 0;
                return true;
            }
            else {
                voidCount++;
            }
        }
        else if (color2 != null) {
            if (color2 != ArtifactColor.NONE) {
                lastColor = color2Value;
                lastArtifactColor = color2;
                voidCount = 0;
                return true;
            }
            else {
                voidCount++;
            }
        }
        else {
            // if both sensors haven't detected any artifact, reset "no color matches" counter
            voidCount = 0;
        }

        // if both sensors haven't matched any color for voidThreshold iterations or more, mark the color as NONE and move on. otherwise just continue scanning
        if (voidCount >= voidThreshold) {
            voidCount = 0;
            lastColor = color1Value;
            lastArtifactColor = ArtifactColor.NONE;
            return true;
        }
        else {
            return false;
        }
    }
    public boolean isIntaking() {
        return isIntaking;
    }

    public boolean isNowStoppedIntaking() {
        return isIntakingLastUpdate && !isIntaking;
    }

    public void startIntaking() {
        isIntaking = true;
        boolean flag = false;
        SpindexerPosition position = SpindexerPosition.INTAKE_INDEX_0;
        while (!flag && storedArtifacts[position.getSelectedArrayIndex()] != null) {
            position = position.getNext();
            flag = position == SpindexerPosition.INTAKE_INDEX_0;
        }

        if (flag) {
            stopIntaking();
        }

        setSpindexerPosition(position);
    }

    public void stopIntaking() {
        isIntaking = false;
    }

    // defines artifact object using lastColor and puts it in the thing\

    private ArtifactColor matchColor(NormalizedRGBA color) {
        double[] lastColorArray = ArtifactColor.NormalizedRGBAToArray(color);
        if (Utils.areNormalizedRGBArraysSimilar(lastColorArray, ArtifactColor.GREEN.color)) {
            return ArtifactColor.GREEN;
        }

        if (Utils.areNormalizedRGBArraysSimilar(lastColorArray, ArtifactColor.PURPLE.color)) {
            return ArtifactColor.PURPLE;
        }

        return ArtifactColor.NONE;
    }
    // returns true if another can be intaked and moves to the next position to intake, false otherwise
    private boolean artifactEntered(ArtifactColor color) {
        storedArtifacts[targetSpindexerPosition.getSelectedArrayIndex()] = color;

        SpindexerPosition nextPosition = targetSpindexerPosition.getNext();

        for (int i = 0; i < 2; i++) {
            if (storedArtifacts[nextPosition.getSelectedArrayIndex()] != null) {
                nextPosition = nextPosition.getNext();
            }
            else {
                setSpindexerPosition(nextPosition);
                return true;
            }
        }
        return false;
    }
    public void setSpindexerPosition(SpindexerPosition position) {
        targetSpindexerPosition = position;
        spindexerServo.setPosition(position.position);
    }
    public void setFeedingServoPosition(FeedingServoPosition position) {
        currentFeedingServoPosition = position;
        feedingServo.setPosition(position.position);
    }

    public FeedingServoPosition getCurrentFeedingServoPosition() {
        return currentFeedingServoPosition;
    }

    // only this should be used to access the encoder's position, as this matches the encoder's reading to the servo's position (they are same when servo isn't moving)
    public double getScaledSpindexerEncoderPosition() {
        return ((spindexerEncoder.getVoltage()/spindexerEncoder.getMaxVoltage()) - analogInputMin)/(analogInputMax - analogInputMin);
    }

    public boolean isSpindexerInPosition() {
        return Math.abs(getScaledSpindexerEncoderPosition() - targetSpindexerPosition.position) <= spindexerEncoderTolerance;
    }

    private int countArtifactsInSpindexer() {
        int sum = 0;
        for (int i = 0; i < storedArtifacts.length; i++) {
            if (storedArtifacts[i] != null) {
                sum++;
            }
        }
        return sum;
    }

    private int getArtifactColorCount(ArtifactColor color) {
        int sum = 0;
        for (int i = 0; i < storedArtifacts.length; i++) {
            if (storedArtifacts[i] == color || (storedArtifacts[i] == ArtifactColor.NONE && color == ArtifactColor.PURPLE)) {
                sum++;
            }
        }
        return sum;
    }

    private int[] getArtifactColorPositions(ArtifactColor color) {
        int[] positions = new int[getArtifactColorCount(color)];
        int positionsIndex = 0;
        for (int i = 0; i < storedArtifacts.length; i++) {
            if (storedArtifacts[i] == color || (storedArtifacts[i] == ArtifactColor.NONE && color == ArtifactColor.PURPLE)) {
                positions[positionsIndex] = i;
                positionsIndex++;
            }
        }
        return positions;
    }

    public SpindexerPosition[] getShootingSequence() {
        SpindexerPosition[] result = new SpindexerPosition[countArtifactsInSpindexer()];

        if (motif == Motif.NONE) {
            SpindexerPosition currIndexStored;
            if (targetSpindexerPosition == SpindexerPosition.SHOOT_INDEX_0 || targetSpindexerPosition == SpindexerPosition.SHOOT_INDEX_1 || targetSpindexerPosition == SpindexerPosition.SHOOT_INDEX_2) {
                currIndexStored = targetSpindexerPosition;
            }
            else {
                currIndexStored = SpindexerPosition.SHOOT_INDEX_0;
            }
            for (int i = 0; i < result.length; i++) {
                while (storedArtifacts[currIndexStored.getSelectedArrayIndex()] == null) {
                    currIndexStored = currIndexStored.getNext();
                }
                result[i] = currIndexStored;
                currIndexStored = currIndexStored.getNext();
            }
        }
        else {
            int[] purpleArtifactPositions = getArtifactColorPositions(ArtifactColor.PURPLE);
            int[] greenArtifactPositions = getArtifactColorPositions(ArtifactColor.GREEN);
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

    public SpindexerPosition getTargetSpindexerPosition() {
        return targetSpindexerPosition;
    }

    public String getStoredArtifacts() {
        return "[" + storedArtifacts[0] + ", " + storedArtifacts[1] + ", " + storedArtifacts[2] + "]";
    }

    public FeedSingleArtifactAction getFeedSingleArtifactAction(SpindexerPosition positionToShoot) {
        return new FeedSingleArtifactAction(positionToShoot);
    }

    public MoveSpindexerAction getMoveSpindexerAction(SpindexerPosition position) {
        return new MoveSpindexerAction(position);
    }

    public Action getScanArtifactColorsAction() {
        return new SequentialAction(
                new MoveSpindexerAction(SpindexerPosition.INTAKE_INDEX_0),
                new ScanCurrentArtifactAction(),
                new MoveSpindexerAction(SpindexerPosition.INTAKE_INDEX_1),
                new ScanCurrentArtifactAction(),
                new MoveSpindexerAction(SpindexerPosition.INTAKE_INDEX_2),
                new ScanCurrentArtifactAction()
        );
    }

    // TODO: ALL THOSE BELOW THIS COMMENT ARE TESTS AND SHOULD NOT BE USED

    @Deprecated
    public static int countArtifactsInSpindexer(ArtifactColor[] storedArtifacts) {
        int sum = 0;
        for (int i = 0; i < storedArtifacts.length; i++) {
            if (storedArtifacts[i] != null) {
                sum++;
            }
        }
        return sum;
    }

    @Deprecated
    public static int getArtifactColorCount(ArtifactColor[] storedArtifacts, ArtifactColor color) {
        int sum = 0;
        for (int i = 0; i < storedArtifacts.length; i++) {
            if (storedArtifacts[i] == color || (storedArtifacts[i] == ArtifactColor.NONE && color == ArtifactColor.PURPLE)) {
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
            if (storedArtifacts[i] == color || (storedArtifacts[i] == ArtifactColor.NONE && color == ArtifactColor.PURPLE)) {
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
