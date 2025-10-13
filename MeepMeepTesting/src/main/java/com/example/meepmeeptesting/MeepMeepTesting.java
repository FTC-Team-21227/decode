package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .build();

        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(-55, 46, Math.toRadians(-55)))
                .waitSeconds(5)
                .splineTo(new Vector2d(-55+20,46-20*Math.tan(Math.toRadians(55))),Math.toRadians(-55))
                .splineTo(new Vector2d(-12,52),Math.toRadians(90))
                .strafeToLinearHeading(new Vector2d(-12,15),Math.toRadians(45))
                .setTangent(Math.toRadians(45))
                .splineTo(new Vector2d(12,52),Math.toRadians(90))
                .strafeTo(new Vector2d(-12,15))
                .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_DECODE_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}