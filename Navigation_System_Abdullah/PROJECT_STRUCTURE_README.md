# FYP Simulation Project — Clean Structure

## What to do first
1. Unzip this wherever you want, but the **folder layout inside must map
   exactly onto `C:\Users\User\Documents\`** — i.e. `WEBOT2.O\` and `WEEDS\`
   should sit directly under `Documents\`, matching the paths already baked
   into the plotting scripts. If you unzip somewhere else, move these two
   folders to `Documents\` afterward, or update the paths in the three
   plotting scripts.
2. Go into `WEBOT2.O\worlds\`, delete `PASTE_YOUR_WEEDBOTPRO_MASTER_WBT_HERE.txt`,
   and paste in your real, latest `.wbt` — the one with the green front-marker
   cone on the robot. Rename it `WEEDBOTPRO_master.wbt` if you like (not required).
3. **This is now your only world file, forever.** Never duplicate it per
   experiment again.

## The controllers/ folder confusion (why things got messy before)
Webots only scans the `controllers/` folder that sits **next to** `worlds/`
(both directly under `WEBOT2.O\`). If you ever see a `controllers` folder
*inside* `worlds\`, that one is not used by Webots — delete it, or at least
never put files there.

## How to run each milestone
Open the world once. In the Scene Tree, select the `mecanum_platform` robot
node, and change its `controller` field to the one you want to run:

| Controller folder              | What it does                                          |
|---------------------------------|--------------------------------------------------------|
| `mecanum_boustrophedon`         | Milestone A — basic waypoint following, no logging      |
| `mecanum_trajectory_log`        | Milestone B — adds `trajectory_log.csv` (Fig 6a)         |
| `mecanum_localization`          | Milestone C — adds GPS noise + filter + blackout (Fig 6b)|
| `mecanum_milestoneD_crab`       | Milestone D — crab-walk row switching                    |
| `mecanum_milestoneD_turn`       | Milestone D — turn-in-place row switching                 |
| `mecanum_milestoneD_efficient`  | Milestone D — no-fixed-front (reverse-instead-of-turn)     |

After changing the controller field: **Ctrl+S**, then fully **reset** the
simulation (not resume from pause), then run. Check the console's first
printed line (`[Nav] ROW_SWITCH_MODE = ...` for Milestone D controllers) to
confirm you're running the one you meant to.

Each controller writes into its **own** `results\` subfolder, so none of
them can overwrite each other's CSVs — that was the root cause of a lot of
the earlier mess.

## Where results land
```
WEBOT2.O\controllers\mecanum_trajectory_log\results\trajectory_log.csv
WEBOT2.O\controllers\mecanum_localization\results\trajectory_log.csv
WEBOT2.O\controllers\mecanum_localization\results\localization_log.csv
WEBOT2.O\controllers\mecanum_milestoneD_crab\results\row_switch_log_crab.csv
WEBOT2.O\controllers\mecanum_milestoneD_turn\results\row_switch_log_turn.csv
WEBOT2.O\controllers\mecanum_milestoneD_efficient\results\row_switch_log_efficient.csv
```
(Each Milestone D controller also writes its own `trajectory_log.csv` and
`localization_log.csv` in its own results folder, since the full merged
navigation+logging logic runs in all of them — but for the paper you'll
mainly use the standalone `mecanum_trajectory_log` and `mecanum_localization`
runs for Figures 6a/6b specifically, since those are the "clean" isolated
milestone demonstrations.)

## Plotting scripts (in `WEEDS\Fyp-Project-Paper\Navigation_Subsystem\`)
Run these separately in VS Code once you have the relevant CSVs:

- **`Trajectory-log-Fig6a.py`** → reads `mecanum_trajectory_log\results\trajectory_log.csv`,
  produces `figure_6a_trajectory.png`
- **`Localization-log-Fig6b.py`** → reads `mecanum_localization\results\localization_log.csv`,
  produces `figure_6b_localization_error.png`
- **`plot_row_switch.py`** → reads both `mecanum_milestoneD_crab\results\row_switch_log_crab.csv`
  and `mecanum_milestoneD_turn\results\row_switch_log_turn.csv`, produces three figures
  (`figure_6c1`, `6c2`, `6c3`) into a `Crab vs Turn\` subfolder

All three already have the correct paths pre-filled for this structure — you
should only need to edit them if you move something.

## Known gap to flag
Your originally-uploaded "V3-GPS-Error" file did not actually contain the
GPS noise/filter/blackout logic — `mecanum_localization.py` in this package
was reconstructed from the validated version we built together earlier in
chat, since the upload was actually just Milestone B content under a
misleading filename. Worth a quick skim of `mecanum_localization.py` to
confirm it matches your expectations before you run it.
