package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeRedDark;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class BlueLine15Auto {
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

        /**
         * STEPS:
         * Shoot first 3: 3/15
         * Intake 1st row
         * Shoot 1st row: 6/15
         * Intake 2nd row
         * Shoot 2nd row: 9/15
         * Open gate
         * Intake 3
         * Shoot 3: 12/15
         * Intake 3rd row
         * Shoot 3rd row: 15/15
         */
        redBot.runAction(
                redBot.getDrive().actionBuilder(startPose)
                        // 1st Trajectory: Move out, read obelisk
                        .strafeTo(new Vector2d(-12, 46 - 20 * Math.tan(Math.toRadians(55))))
                        .turnTo(Math.toRadians(180))
                        // TODO: Shoot here. 3/15
                        // Face artifacts
                        .turnTo(Math.toRadians(90))
                        // 2nd Trajectory: collect 2nd row and open gate
                        .strafeTo(new Vector2d(12, 30)) // Line up with artifacts
                        .strafeTo(new Vector2d(12, 45)) // Move forward to collect
                        .strafeTo(new Vector2d(10, 56)) // Open gate
                        .strafeTo(new Vector2d(8, 40)) // Back up (avoid hitting 1st row)
                        .strafeTo(new Vector2d(-12, 15)) // Shooting pos
                        // TODO: Shoot here. 6/15
                        // 3rd Trajectory: collect 1st row
                        .strafeTo(new Vector2d(-12, 45)) // Move forward to collect
                        .strafeTo(new Vector2d(-12, 15)) // Shooting pos
                        // TODO: Shoot here. 9/15
                        // 4th Trajectory: collect 3rd (last) row
                        .strafeTo(new Vector2d(36, 30)) // Line up with artifacts
                        .strafeTo(new Vector2d(36, 45)) // Move forward to collect
                        .strafeTo(new Vector2d(-12, 15)) // Shooting pos
                        // TODO: Shoot here. 12/15
                        // 5th Trajectory: collect 3 from gate release
                        .strafeTo(new Vector2d(20, 58)) // Collect from gate release
                        .strafeTo(new Vector2d(-12, 15)) // Shooting pos
                        // TODO: Shoot here. 15/15
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