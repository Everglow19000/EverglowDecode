package org.firstinspires.ftc.teamcode.tuning;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.teamcode.subsystems.FeedingMechanism;
import org.firstinspires.ftc.teamcode.subsystems.Motif;

import java.util.Arrays;

@TeleOp(name="Feeding Mechanism Testing", group="Tests")
@Config
public class FeedingMechanismTesting extends LinearOpMode {
    FeedingMechanism feedingMechanism;
    public static double spindexerPosition = 0;
    public static double feedingPosition = 0.5;
    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        feedingMechanism = new FeedingMechanism(hardwareMap, Motif.NONE);
        AnalogInput spindexerInput = hardwareMap.get(AnalogInput.class, "spindexerInput");

        waitForStart();


        feedingMechanism.setFeedingServoPosition(FeedingMechanism.FeedingServoPosition.DOWN);

        sleep(400);

        boolean flagCross = false;
        boolean flagCircle = false;
        boolean flagSquare = false;
        boolean flagTriangle = false;

        telemetry.addLine("starting action");
        telemetry.update();

        Actions.runBlocking(feedingMechanism.getScanArtifactColorsAction());

        boolean manual = false;

        while (opModeIsActive()) {
            if (gamepad1.circle && !flagCircle) {
                manual = !manual;
            }
            flagCircle = gamepad1.circle;

            if (manual) {
                feedingMechanism.spindexerServo.setPosition(spindexerPosition);
                feedingMechanism.feedingServo.setPosition(feedingPosition);
            }
            else {
                if (gamepad1.cross && !flagCross) {
                    feedingMechanism.setSpindexerPosition(feedingMechanism.getCurrentSpindexerPosition() == FeedingMechanism.SpindexerPosition.INTAKE_INDEX_2 ? FeedingMechanism.SpindexerPosition.SHOOT_INDEX_0 : feedingMechanism.getCurrentSpindexerPosition() == FeedingMechanism.SpindexerPosition.SHOOT_INDEX_2 ? FeedingMechanism.SpindexerPosition.INTAKE_INDEX_0 : feedingMechanism.getCurrentSpindexerPosition().getNext());
                }
                flagCross = gamepad1.cross;
                if (gamepad1.square && !flagSquare) {
                    feedingMechanism.setFeedingServoPosition(feedingMechanism.getCurrentFeedingServoPosition().toggle());
                }
                flagSquare = gamepad1.square;
            }


            telemetry.addData("curr pos", feedingMechanism.getCurrentSpindexerPosition().name());
            telemetry.addData("pos value", feedingMechanism.getCurrentSpindexerPosition().position);
            telemetry.addData("contents", feedingMechanism.getStoredArtifacts());
            telemetry.update();
        }
    }
}
