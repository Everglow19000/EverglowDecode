package org.firstinspires.ftc.teamcode.tuning;

import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.teamcode.subsystems.FeedingMechanism;
import org.firstinspires.ftc.teamcode.subsystems.Motif;

import java.util.Arrays;

@TeleOp(name="Feeding Mechanism Testing")
public class FeedingMechanismTesting extends LinearOpMode {
    FeedingMechanism feedingMechanism;
    @Override
    public void runOpMode() throws InterruptedException {
        feedingMechanism = new FeedingMechanism(hardwareMap, Motif.NONE);



        waitForStart();

        sleep(10000);
//
//        Actions.runBlocking(feedingMechanism.getScanArtifactColorsAction());

        Actions.runBlocking(feedingMechanism.getFeedSingleArtifactAction(FeedingMechanism.SpindexerPosition.SHOOT_INDEX_0));

        while (opModeIsActive()) {
            telemetry.addData("limit", feedingMechanism.limitSwitch.getState());
            telemetry.addData("stored artifacts", feedingMechanism.getStoredArtifacts());
            telemetry.addData("position", feedingMechanism.getCurrentSpindexerPosition().name());
            telemetry.update();
        }
    }
}
