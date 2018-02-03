//      Conic State Extrapolation
// Formulas follow H.D. Curtis, Orbital Mechanics for Engineering Students, Chapter 3.7
// Radius vector scaled by its magnitude, velocity scaled by circular speed sqrt(mu/ r0)
// With this scaling, mu_scaled = 1, so no need to bother about sqrt(mu) or 1 / mu etc.

// Stumpff S and C functions
FUNCTION SnC {
        DECLARE PARAMETER z.
        LOCAL az IS ABS(z).
        IF az < 1e-4 {
                RETURN LEXICON("S", (1 - z * ( 0.05 - z / 840) ) / 6, "C", 0.5 - z * ( 1 - z / 30) / 24).
        }
        ELSE {
                LOCAL saz IS SQRT(az).
                IF z > 0 {
                        LOCAL x IS saz * CONSTANT:RADTODEG.
                        RETURN LEXICON("S", (saz - SIN(x)) / (saz * az), "C", (1 - COS(x)) / az).
                }
                ELSE {
                        LOCAL x IS CONSTANT:E^saz.
                        RETURN LEXICON("S", (0.5 * (x - 1 / x) - saz) / (saz * az), "C", (0.5 * (x + 1 / x) - 1) / az).
                }
        }
}

// Conic State Extrapolation Routine
FUNCTION cser {
        DECLARE PARAMETER r0, v0, dt, mu IS BODY:MU, x0 IS 0, tol IS 5e-9.
        LOCAL rscale IS r0:mag.
        LOCAL vscale IS SQRT(mu / rscale).
        LOCAL r0s IS r0 / rscale.
        LOCAL v0s IS v0 / vscale.
        LOCAL dts IS dt * vscale / rscale.
        LOCAL v2s IS v0:SQRMAGNITUDE * rscale / mu.
        LOCAL alpha IS 2 - v2s.
        LOCAL armd1 IS v2s - 1.
        LOCAL rvr0s IS VDOT(r0, v0) / SQRT(mu * rscale).

        LOCAL x IS x0.
        IF x0 = 0 { SET x TO dts * abs(alpha). }
        LOCAL ratio IS 1.
        LOCAL x2 IS x * x.
        LOCAL z IS alpha * x2.
        LOCAL SCz IS SnC(z).
        LOCAL x2Cz IS x2 * SCz["C"].
        LOCAL f IS 0.
        LOCAL df IS 0.

        UNTIL ABS(ratio) < tol {
                SET f TO x + rvr0s * x2Cz + armd1 * x * x2 * SCz["S"] - dts.
                SET df TO x * rvr0s * (1 - z * SCz["S"]) + armd1 * x2Cz + 1.
                SET ratio TO f / df.
                SET x TO x - ratio.
                SET x2 to x * x.
                SET z TO alpha * x2.
                SET SCz TO SnC(z).
                SET x2Cz TO x2 * SCz["C"].
        }

        LOCAL Lf IS 1 - x2Cz.
        LOCAL Lg IS dts - x2 * x * SCz["S"].

        LOCAL r1 IS Lf * r0s + Lg * v0s.
        LOCAL ir1 IS 1 / r1:MAG.
        LOCAL Lfdot IS ir1 * x * (z * SCz["S"] - 1).
        LOCAL Lgdot IS 1 - x2Cz * ir1.

        LOCAL v1 IS Lfdot * r0s + Lgdot * v0s.

        RETURN LIST(r2 * rscale, v1 * vscale, x).
}
