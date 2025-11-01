package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeRedDark;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class GRAYredAutoMeep {
    public static void main(String[] args) {

        MeepMeep meepMeep = new MeepMeep(800);

        // === Define our red-side bot ===
        RoadRunnerBotEntity redBot = new DefaultBotBuilder(meepMeep)
                .setColorScheme(new ColorSchemeRedDark())
                .setConstraints(
                        60, 60,            // max vel, max accel
                        Math.toRadians(180), Math.toRadians(180), 15 // max ang vel, ang accel, track width
                )
                .build();

        // === Follow the same positions from your Auto_Red_Goal.java ===
        Pose2d startPose = new Pose2d(-55, 46, Math.toRadians(-55));

        redBot.runAction(
                redBot.getDrive().actionBuilder(startPose)
                        // 1st Trajectory: Move out, read obelisk
                        .strafeTo(new Vector2d(-12, 46 - 20 * Math.tan(Math.toRadians(55))))
                        .turnTo(Math.toRadians(180))
                        // TODO: Shoot here. 3/12
                        // Face artifacts
                        .turnTo(Math.toRadians(90))
                        // 2nd Trajectory: collect 1st row
                        .strafeTo(new Vector2d(-12, 45)) // Move forward
                        .strafeTo(new Vector2d(-12, 15)) // Shooting pos
                        // TODO: Shoot here. 6/12
                        // Third Trajectory: collect 2nd row
                        .strafeTo(new Vector2d(12, 15)) // Line up with artifacts
                        .strafeTo(new Vector2d(12, 45)) // Move forward
                        .strafeTo(new Vector2d(-12, 15)) // Shooting pos
                        // TODO: Shoot here. 9/12
                        // Fourth Trajectory: collect 3rd (last) row
                        .strafeTo(new Vector2d(36, 15)) // Line up with artifacts
                        .strafeTo(new Vector2d(36, 45)) // Move forward
                        .strafeTo(new Vector2d(-12, 15)) // Shooting pos
                        // TODO: Shoot here. 12/12
                        .build()
        );

        // MeepMeep settings
        meepMeep.setBackground(MeepMeep.Background.FIELD_DECODE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(redBot)
                .start();
    }
}