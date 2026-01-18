package com.example.meepmeep;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeBlueLight;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class close_test {
    public static void main(String[] args) throws InterruptedException {
    MeepMeep meepMeep = new MeepMeep(700);

    RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
            .setColorScheme(new ColorSchemeBlueLight())
            // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
            .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(1.80), 15)
            .setDimensions(18,18)
            .build();


    myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(-50, -48, Math.toRadians(225)))
                    .setTangent(Math.toRadians(45))
                    .waitSeconds(8)
                            .splineToSplineHeading(new Pose2d(-36, 18, Math.toRadians(-135)), Math.PI/2.0)
                            .waitSeconds(1)
                    //.strafeTo(new Vector2d(-30,-28))
            //.waitSeconds(1)
            //.setTangent(Math.toRadians(45))

            .splineToSplineHeading(new Pose2d(-12,-30, Math.toRadians(-90)) ,Math.toRadians(-90))
            .splineTo(new Vector2d(-12,-40) ,Math.toRadians(-90))
                            .splineTo(new Vector2d(-30,-28), Math.toRadians(225))
            .waitSeconds(8)
                    .strafeTo(new Vector2d(0,-28))
            .build());

    meepMeep.setBackground(MeepMeep.Background.FIELD_DECODE_JUICE_DARK)
            .setDarkMode(true)
            .setBackgroundAlpha(0.95f)
            .addEntity(myBot)
            .start();
    }
}

