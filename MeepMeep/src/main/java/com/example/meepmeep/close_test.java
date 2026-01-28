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
            .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(1.80), 15)
            .setDimensions(18,18)
            .build();


    myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(-48, -48 * isBlueValue, Math.toRadians(90 * isBlueValue)))
            .setTangent(0)
            .splineToSplineHeading(new Pose2d(-12, -30 * isBlueValue, Math.toRadians(-90 * isBlueValue)), Math.toRadians(-90 * isBlueValue))
            .splineTo(new Vector2d(-12, -55 * isBlueValue), Math.toRadians(-90 * isBlueValue), new TranslationalVelConstraint(10))
            .waitSeconds(2)
            .setTangent(Math.toRadians(90 * isBlueValue))
            .splineToSplineHeading(new Pose2d(-30, -28 * isBlueValue, Math.toRadians(-135*isBlueValue)), Math.toRadians(180))
            .waitSeconds(2)
            .setTangent(0)
            .splineToSplineHeading(new Pose2d(0, -48 * isBlueValue, Math.toRadians(90 * isBlueValue)), -(Math.PI/2.0) * isBlueValue)
            .build());

    meepMeep.setBackground(MeepMeep.Background.FIELD_DECODE_JUICE_DARK)
            .setDarkMode(true)
            .setBackgroundAlpha(0.95f)
            .addEntity(myBot)
            .start();
    }
}

