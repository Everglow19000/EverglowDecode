package org.firstinspires.ftc.teamcode.tuning;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;

import org.firstinspires.ftc.teamcode.subsystems.FeedingMechanism;
import org.firstinspires.ftc.teamcode.subsystems.Motif;

@TeleOp(name="Feeding Mechanism Testing", group="Tests")
@Config
public class FeedingMechanismTesting extends LinearOpMode {
    FeedingMechanism feedingMechanism;
    public static double spindexerPosition = 0;
    public static double feedingPosition = 0.5;
    public static boolean manual = true;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        feedingMechanism = new FeedingMechanism(hardwareMap, Motif.NONE);
        AnalogInput spindexerInput = hardwareMap.get(AnalogInput.class, "spindexerInput");

        waitForStart();


        feedingMechanism.setFeedingServoPosition(FeedingMechanism.FeedingServoPosition.DOWN);


        boolean flagCross = false;
        boolean flagCircle = false;
        boolean flagSquare = false;
        boolean flagTriangle = false;

        telemetry.addLine("scanning action started");
        telemetry.update();

        Actions.runBlocking(feedingMechanism.getScanArtifactColorsAction());

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
                    feedingMechanism.setSpindexerPosition(feedingMechanism.getTargetSpindexerPosition() == FeedingMechanism.SpindexerPosition.INTAKE_INDEX_2 ? FeedingMechanism.SpindexerPosition.SHOOT_INDEX_0 : feedingMechanism.getTargetSpindexerPosition() == FeedingMechanism.SpindexerPosition.SHOOT_INDEX_2 ? FeedingMechanism.SpindexerPosition.INTAKE_INDEX_0 : feedingMechanism.getTargetSpindexerPosition().getNext());
                }
                flagCross = gamepad1.cross;
                if (gamepad1.square && !flagSquare) {
                    feedingMechanism.setFeedingServoPosition(feedingMechanism.getCurrentFeedingServoPosition().toggle());
                }
                flagSquare = gamepad1.square;
            }

            telemetry.addData("is in position", feedingMechanism.isSpindexerInPosition());
            telemetry.addData("limit switch sensing", !feedingMechanism.limitSwitch.getState());
            telemetry.addData("analog spindexer pos", spindexerInput.getVoltage()/spindexerInput.getMaxVoltage());
            telemetry.addData("curr spindexer pos", feedingMechanism.getTargetSpindexerPosition().name());
            telemetry.addData("spindexer pos value", feedingMechanism.getTargetSpindexerPosition().position);
            telemetry.addData("contents", feedingMechanism.getStoredArtifacts());
            telemetry.update();
        }
    }
}
