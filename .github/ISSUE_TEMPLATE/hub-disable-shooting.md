---
title: "Disable Shooting When Hub Inactive (2026 Game Data Integration)"
labels: [game-data, shooting, bug, enhancement]
---

## Summary
Implements logic to **disable shooting whenever our alliance's hub is inactive** during the 2026 FRC game, as determined by FMS Game Data and match time. This prevents wasting fuel and battery when scoring would not earn points.

## Details
- Uses `DriverStation.getGameSpecificMessage()` and `DriverStation.getMatchTime()` to determine hub status per 2026 rules.
- New method `isHubActive()` in `HubTargetingSubsystem` encapsulates the logic (see WPILib docs: [2026 Game Data](https://docs.wpilib.org/en/stable/docs/yearly-overview/2026-game-data.html)).
- `ShootSequence` will not spin up or feed balls if the hub is inactive.
- Shuffleboard now displays a `Hub Active` status for driver awareness.

## How to Test
1. **Simulate Game Data:**
   - Open the Driver Station, go to the Setup tab.
   - Enter `R` or `B` in the Game Data field (`R` = Red hub inactive first, `B` = Blue hub inactive first).
2. **Practice Match:**
   - Use the Practice Match feature to run a full match cycle (set times to 20s Auto, 110s Teleop, 30s End Game).
   - Observe the `Hub Active` indicator on Shuffleboard.
   - Attempt to shoot during each shift:
     - When `Hub Active` is **false**, the shooter and feeder should not run.
     - When `Hub Active` is **true**, normal shooting is allowed.
3. **Verify Edge Cases:**
   - Test with no game data (should default to allowing shooting in transition/endgame).
   - Test with corrupt/invalid data (should fail open and allow shooting).

## References
- [2026 Game Data Details (WPILib)](https://docs.wpilib.org/en/stable/docs/yearly-overview/2026-game-data.html)
- FRC Game Manual, Section 6.4/6.5

---
**This issue documents the new safety logic for 2026 and how to verify it in simulation and practice.**
