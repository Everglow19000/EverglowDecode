package com.example.meepmeep;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeBlueLight;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeRedLight;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class close_test {
    public static void main(String[] args) throws InterruptedException {
    MeepMeep meepMeep = new MeepMeep(700);

    boolean isBlue = true;
    int isBlueValue = isBlue ? 1 : -1;

    RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
            .setColorScheme(isBlue ? new ColorSchemeBlueLight() : new ColorSchemeRedLight())
            // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
            .setConstraints(60, 60, Math.PI, Math.PI, 15)
            .setDimensions(18,18)
            .build();


        Vector2d obeliskScanPosition = new Vector2d(-24, -18 * isBlueValue);

        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(-36, -52 * isBlueValue, Math.toRadians(90 * isBlueValue)))
                // scan content, detect motif
                .splineToSplineHeading(new Pose2d(obeliskScanPosition, Math.toRadians(150 * isBlueValue)), Math.toRadians(0)) // while doing this and the next action, spin up shooter, scan artifacts inside, and scan motif.
                .turnTo(Math.toRadians(-135 * isBlueValue))
                .setTangent(0)

                // shooting and end of content scanning
                .waitSeconds(3)

                // intake next 3 artifacts
                .splineToSplineHeading(new Pose2d(-12, -30 * isBlueValue, Math.toRadians(-90 * isBlueValue)), Math.toRadians(-90 * isBlueValue))
                .splineTo(new Vector2d(-12, -45 * isBlueValue), Math.toRadians(-90 * isBlueValue), new TranslationalVelConstraint(20))

                // open gate
                .splineToSplineHeading(new Pose2d(-5, -52 * isBlueValue, Math.toRadians(90 * isBlueValue)), Math.toRadians(-45 * isBlueValue))
                .strafeTo(new Vector2d(-5, -56))
                .waitSeconds(2)

                // shoot
                .splineToSplineHeading(new Pose2d(obeliskScanPosition, Math.toRadians(-135 * isBlueValue)), Math.toRadians(135 * isBlueValue))
                .waitSeconds(3)

                // intake next 3 artifacts
                .setTangent(0)
                .splineToSplineHeading(new Pose2d(12, -30 * isBlueValue, Math.toRadians(-90 * isBlueValue)), Math.toRadians(-90 * isBlueValue))
                .splineTo(new Vector2d(12, -45 * isBlueValue), Math.toRadians(-90 * isBlueValue), new TranslationalVelConstraint(20))

                // shoot
                .splineToSplineHeading(new Pose2d(obeliskScanPosition, Math.toRadians(-135 * isBlueValue)), Math.toRadians(135 * isBlueValue))
                .waitSeconds(3)

                // move for leave points
                .splineTo(new Vector2d(-56, -24), Math.toRadians(180))
            .build());

    meepMeep.setBackground(MeepMeep.Background.FIELD_DECODE_JUICE_DARK)
            .setDarkMode(true)
            .setBackgroundAlpha(0.95f)
            .addEntity(myBot)
            .start();
    }
}

