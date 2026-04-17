package org.firstinspires.ftc.teamcode;

import java.util.Map;

public class MotifStorage {
    public enum MotifState {
        GPP,
        PGP,
        PPG
    }

    public Map<Integer, MotifState> motifPoses = Map.of(
        21, MotifState.GPP,
        22, MotifState.PGP,
        23, MotifState.PPG
    );

    public static MotifState currentMotif = MotifState.GPP;

    public MotifState getMotifState(int id) {
        return motifPoses.getOrDefault(id, MotifState.GPP);
    }
}
