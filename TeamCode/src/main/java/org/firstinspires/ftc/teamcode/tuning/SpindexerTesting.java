package org.firstinspires.ftc.teamcode.tuning;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.everglow_library.Utils;
import org.firstinspires.ftc.teamcode.subsystems.ArtifactColor;
import org.firstinspires.ftc.teamcode.subsystems.FeedingMechanism;
import org.firstinspires.ftc.teamcode.subsystems.Motif;

import java.util.Arrays;

@TeleOp(name="SpindexerTesting")
@Config
public class SpindexerTesting extends LinearOpMode {

    public static int motifIndex = 0;
    public static int storedArtifactIndex0 = 0;
    public static int storedArtifactIndex1 = 0;
    public static int storedArtifactIndex2 = 0;
    public static int startingSpindexerPosition = 0;
    public static boolean feedingCycleActive = false;
    public static double feedingCycleTimeMS = 200;
    public static boolean spindexerActive = false;
    public static double spindexerPosition = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        Servo feedingServo = hardwareMap.get(Servo.class, "feedingServo");
        feedingServo.setDirection(Servo.Direction.REVERSE);

        Servo spindexerServo = hardwareMap.get(Servo.class, "spindexerServo");
        Utils.setServoPWMRange(spindexerServo, 510, 2490);

        Motif[] motifs = new Motif[4];
        motifs[0] = Motif.NONE;
        motifs[1] = Motif.PGP;
        motifs[2] = Motif.GPP;
        motifs[3] = Motif.PPG;

        ArtifactColor[] artifactColors = new ArtifactColor[3];
        artifactColors[0] = ArtifactColor.NONE;
        artifactColors[1] = ArtifactColor.PURPLE;
        artifactColors[2] = ArtifactColor.GREEN;

        FeedingMechanism.SpindexerPosition[] spindexerPositions = new FeedingMechanism.SpindexerPosition[3];
        spindexerPositions[0] = FeedingMechanism.SpindexerPosition.SHOOT_INDEX_0;
        spindexerPositions[1] = FeedingMechanism.SpindexerPosition.SHOOT_INDEX_1;
        spindexerPositions[2] = FeedingMechanism.SpindexerPosition.SHOOT_INDEX_2;

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        double servoPos = FeedingMechanism.FeedingServoPosition.UP.position;
        waitForStart();

        while (opModeIsActive()) {
            if (feedingCycleActive && (System.currentTimeMillis() % (feedingCycleTimeMS*2)) - feedingCycleTimeMS >= 0) {
                servoPos = FeedingMechanism.FeedingServoPosition.UP.position;
            }
            else {
                servoPos = FeedingMechanism.FeedingServoPosition.DOWN.position;
            }

            if (spindexerActive) {
                spindexerServo.setPosition(spindexerPosition);
            }

            feedingServo.setPosition(servoPos);

            ArtifactColor[] colors = new ArtifactColor[3];
            colors[0] = artifactColors[storedArtifactIndex0];
            colors[1] = artifactColors[storedArtifactIndex1];
            colors[2] = artifactColors[storedArtifactIndex2];

            Motif motif = motifs[motifIndex];

            FeedingMechanism.SpindexerPosition[] result = FeedingMechanism.getShootingSequence(motif, colors, spindexerPositions[startingSpindexerPosition]);

            StringBuilder sequenceColors = new StringBuilder();

            for (int i = 0; i < result.length; i++) {
                sequenceColors = sequenceColors.append(colors[result[i].getSelectedArrayIndex()].name().charAt(0));
            }

            telemetry.addData("color 0", artifactColors[storedArtifactIndex0]);
            telemetry.addData("color 1", artifactColors[storedArtifactIndex1]);
            telemetry.addData("color 2", artifactColors[storedArtifactIndex2]);
            telemetry.addData("motif", motif.name());
            telemetry.addData("spindexer position", spindexerPositions[startingSpindexerPosition]);
            telemetry.addData("stored", Arrays.toString(colors));
            telemetry.addData("sequence in colors", sequenceColors.toString());
            telemetry.addData("sequence", Arrays.toString(result));
            telemetry.update();
        }
    }
}
