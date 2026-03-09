package frc.robot.PlayingField;

import java.util.Optional;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public enum MatchPeriod {
    /* From Game Manual Section 6.4
     * Auto Length: 20 seconds
     * Teleop Length (including shifts and endgame): 2 minutes and 20 seconds <===> 140 total seconds
     * Total Match Length: 2 minutes and 40 seconds <===> 160 total seconds
     * 
     * Part Of Match | Duration (seconds) | Timer Values (seconds)
     * --------------+--------------------+-----------------------
     *      AUTO     |         20         |       20 -> 0
     * --------------+--------------------+-----------------------
     *   Transition  |         10         |      140 -> 130
     * --------------+--------------------+-----------------------
     *     Shift 1   |         25         |      130 -> 105
     * --------------+--------------------+-----------------------
     *     Shift 2   |         25         |      105 -> 80
     * --------------+--------------------+-----------------------
     *     Shift 3   |         25         |       80 -> 55
     * --------------+--------------------+-----------------------
     *     Shift 4   |         25         |       55 -> 30
     * --------------+--------------------+-----------------------
     *    End Game   |         30         |       30 -> 0
     * --------------+--------------------+-----------------------
    */
    AUTO(20, 0), TRANSITION_SHIFT(140, 130),
    SHIFT_1(130, 105), SHIFT_2(105, 80),
    SHIFT_3(80, 55), SHIFT_4(55, 30),
    END_GAME(30, 0);

    // Intended to be updated only once at the begining of each robot loop,
    // so that the passage of time within a robot loop doesn't cause different
    // parts of the code to believe we are at different stages within the match.
    // We want each iteration of the main loop to act as a snapshot of an instant
    // in time. We don't want two robot mechanisms following two different gameplans.
    private static double secondsLeftInMatch = -1;

    private final double matchClockSecondsAtStart;
    private final double matchClockSecondsAtEnd;

    private MatchPeriod(double start, double end) {
        this.matchClockSecondsAtStart = start;
        this.matchClockSecondsAtEnd = end;
    }

    private boolean isCurrentPeriod() {
        if (this == MatchPeriod.AUTO) {
            return DriverStation.isAutonomousEnabled();
        }

        // Match clock counts down,
        // meaning [periodStart > periodEnd]
        return (this.matchClockSecondsAtStart >= secondsLeftInMatch) && (secondsLeftInMatch > this.matchClockSecondsAtEnd);
    }

    public static void updateCurrentMatchPeriod() {
        MatchPeriod.secondsLeftInMatch = DriverStation.getMatchTime();

        // Record for the logs
        String currentPeriod = "UNKNOWN";
        if (getCurrentMatchPeriod().isPresent()) {
            currentPeriod = getCurrentMatchPeriod().get().name();
        } 
        Logger.recordOutput("MatchProgress/currentPeriod", currentPeriod);
        Logger.recordOutput("MatchProgress/secondsLeftInMatch", secondsLeftInMatch);
        Logger.recordOutput("MatchProgress/secondsUntilAllowedToScore", secondsUntilAllowedToScore());

        String winnerOfAuto = "UNKNOWN";
        if (getWinnerOfAuto().isPresent()) {
            winnerOfAuto = getWinnerOfAuto().get().name();
        }
        Logger.recordOutput("MatchProgress/winnerOfAuto", winnerOfAuto);
    }

    public static Optional<MatchPeriod> getCurrentMatchPeriod() {
        for (MatchPeriod period : MatchPeriod.values()) {
            if (period.isCurrentPeriod()) {
                return Optional.of(period);
            }
        }
        return Optional.empty();
    }

    public static Optional<Alliance> getWinnerOfAuto() {
        // See: https://docs.wpilib.org/en/stable/docs/yearly-overview/2026-game-data.html
        String allianceThatWonAuto = DriverStation.getGameSpecificMessage();
        if (!allianceThatWonAuto.isEmpty() && allianceThatWonAuto.charAt(0) == 'R') {
            return Optional.of(Alliance.Red);
        }
        if (!allianceThatWonAuto.isEmpty() && allianceThatWonAuto.charAt(0) == 'B') {
            return Optional.of(Alliance.Blue);
        }
        return Optional.empty();
    }

    public static double secondsUntilAllowedToScore() {
        Optional<MatchPeriod> currentPeriod = MatchPeriod.getCurrentMatchPeriod();

        // Default to assuming we can score if we can't tell what period it is.
        if (currentPeriod.isEmpty()) {
            return 0;
        }

        // We can score in all of these periods regardless
        // of what alliance we're on.
        if (currentPeriod.get() == AUTO || currentPeriod.get() == TRANSITION_SHIFT || currentPeriod.get() == END_GAME) {
            return 0;
        }

        // At this point, we need to determine which shifts
        // are our active shifts.
        // See: https://docs.wpilib.org/en/stable/docs/yearly-overview/2026-game-data.html
        Optional<Alliance> winnerOfAuto = MatchPeriod.getWinnerOfAuto();
        if (winnerOfAuto.isEmpty()) {
            // Assume that we can shoot in the event that we're
            // not sure who won auto.
            return 0;
        }

        Optional<Alliance> ourAlliance = DriverStation.getAlliance();
        if (ourAlliance.isEmpty()) {
            // Assume that we can shoot if we're not sure what alliance
            // we're on.
            return 0;
        }

        // We don't have to wait to score in our active shifts
        boolean weWonAuto = ourAlliance.get() == winnerOfAuto.get();
        boolean canScoreNow = weWonAuto && (currentPeriod.get() == SHIFT_2 || currentPeriod.get() == SHIFT_4);
        canScoreNow |= (!weWonAuto) && (currentPeriod.get() == SHIFT_1 || currentPeriod.get() == SHIFT_3);
        if (canScoreNow) {
            return 0;
        }

        // Match clock counts down. [periodStart > periodEnd]
        return MatchPeriod.secondsLeftInMatch - currentPeriod.get().matchClockSecondsAtEnd;
    }
}
