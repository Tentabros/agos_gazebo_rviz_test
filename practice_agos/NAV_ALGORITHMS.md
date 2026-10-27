# Driving forwards into new pipe: the navigation algorithms

AGOS used to explore new pipe in reverse whenever that was shorter. Its
RGB-D camera faces forwards, so it then mapped that pipe with the LiDAR
alone, and RTAB-Map later failed to recognise places it had first seen
facing the other way. The robot now follows one rule:

**Drive forwards into unmapped pipe, reverse only over pipe already mapped,
and turn round only in manholes.**

Five algorithms carry the rule (the numbers are those of the research list,
`navigation_algorithms.md`):

| # | Algorithm | What it does here | Where |
|---|---|---|---|
| A1 | Hybrid-A* (Dolgov et al., IJRR 2010) | The global planner plans the heading as well as the position, forwards only (Dubins curves), so every path leaves the way the robot faces | `config/nav2_agos.yaml`, planner `GridBased` |
| A3 | Reverse only over mapped space | Goals the robot may reverse to are planned through mapped cells only, with a controller that may reverse; goals into new pipe use the forwards-only pair | planner `Mapped`, controller `Reverse`; `explorer.py`, `DRIVE` |
| A4 | Direction-following MPPI (Williams et al., ICRA 2017) | The default controller never reverses (`vx_min: 0`) and follows the direction of the planned path (`PathAngleCritic` mode 2, `PreferForwardCritic`) | controller `FollowPath` |
| A7 | Heading-aware frontier cost (the yaw term of FUEL, Zhou et al., RA-L 2021) | Each frontier costs the drive there plus 2 m per half turn, and says how the robot gets there facing it; a frontier reachable only in reverse is never chosen | `frontiers.py`, `Snapshot.approach` |
| B2 | Global tour (TARE, Cao et al., RSS 2021; FUEL) | Full mode: the next frontier is the first stop of the shortest tour through all of them and back home, not the nearest one | `frontiers.py`, `plan_tour` |
| B1 | Pipe graph (Choset and Burdick, IJRR 2000) | The map as manholes (nodes) and the pipes between them (edges), with dead ends and pipes not yet explored to the end | `frontiers.py`, `PipeGraph` |

## How the robot reaches a frontier facing it

The robot can turn on the spot only where nothing is within 0.5 m of its
centre (its corners are 0.40 m out, and a 900 mm pipe leaves 0.45 m). Those
places are the manholes (`Snapshot.turn_spots`, `Snapshot.manholes`).

For every frontier, `Snapshot.approach` looks at the direction in which the
route to it leaves the robot and picks the cheapest of:

| How | When | Cost |
|---|---|---|
| ahead | the route leaves within 90 deg of where the robot faces (45 deg in a manhole) | route length |
| turn | the robot is in a manhole: turn on the spot first | route length + 2 m x (turn / 180 deg) |
| back | the route passes through a manhole: back up to it along mapped pipe, turn there | route length + 2 m |
| loop | a manhole ahead: drive on to it, turn, come back | route length + 2 x distance to it + 2 m |
| reverse | none of these | never chosen |

The explorer then drives in steps: for `back` it first reverses to the
manhole (state "to a manhole to turn", planner `Mapped`, controller
`Reverse`), turns on the spot to face the route (Nav2's spin), and drives
to the frontier forwards (`GridBased`, `FollowPath`). The planner and
controller are chosen for each goal on Nav2's `/planner_selector` and
`/controller_selector` topics, which the stock behaviour tree listens to.

Driving back to the start at the end of a leg, and home at the end of full
mode, stays in reverse over the mapped pipe: there is nothing new to see,
and turning round would need a manhole.

## The tour (full mode)

`plan_tour` solves the travelling-salesman tour exactly (Held-Karp) over the
9 nearest frontiers and the start: the cost between two frontiers is the
path along the mapped pipes plus a turn, and the tour ends at home. TAD
(F = d + a + t, as before) then picks among the first stops whose tour is
within `reach` (2 m) of the best one. The rules of full mode (keep to the
pipe the robot is in, look into a manhole's other pipes first) still apply,
so in this network they usually leave only one or two frontiers to choose
from, and the tour changes little (results below).

## The pipe graph

`PipeGraph` turns the map into manholes and the pipes between them, with
their lengths, every dead end, and every pipe not yet explored to its end.
It is recomputed every 10 s. RViz shows it on the Explorer display
(manholes blue, pipes purple, and the tour magenta), and the mission report
lists it under "Pipe network". The explorer itself still drives from
frontier to frontier; the graph is the map of the network it leaves behind.

## Turning it off

Both are explorer parameters, so they work with `./agos explore`:

    ./agos explore forward_only:=false   # the old behaviour: may reverse into new pipe
    ./agos explore full tour:=false      # full mode without the tour

`forward_only:=false` uses the old planner (Smac 2D through unknown space,
`GridBased2D`) with the controller that may reverse, and the old frontier
choice, so the two can be compared run against run.

## Results

The mission report now says how far the robot reversed, and what for:

    - Reversed: 49.0 m in all; of that 0.0 m back to a manhole to turn round
      and 0.0 m on the way to a frontier

**Gazebo, the 25 m mission** (`./agos explore`, agos_sewer_network): completed
in 347 s of simulation time, 98.5 m driven (97.6 m before the change). Both
legs were explored forwards; the robot turned 180 deg in the start manhole
between them. All 49.0 m of reversing was the two drives back to the start
over mapped pipe; none was into new pipe. The map was 0.04 m off at the end.

**Offline, the whole network** (the explorer's own decision code driving a
simulated LiDAR through the network's geometry; all 9 pipe runs mapped in
each case):

| | Before | Forward only | Forward only + tour |
|---|---|---|---|
| Distance driven | 305 m | 335 m | 335 m |
| Reversed in all | 166 m | 57 m | 57 m |
| Reversed within 3 m of a frontier | 52 m | 0 m | 0 m |
| Frontiers reached in reverse | 5 | 0 | 0 |
| Turns in a manhole | 0 | 7 | 7 |

Driving forwards costs 10 % more distance (the trips to a manhole to turn),
and saves two thirds of the reversing. The tour made no difference in this
network, because the full-mode rules leave it little choice.

**Gazebo, the whole network** (`./agos explore full`, with SLAM on a stand-in
Raspberry Pi through agos_pi_slam): completed in 995 s of simulation time,
267.1 m driven, and the map was 0.15 m off at the end. It reversed 86.4 m:
84.2 m back to a manhole to turn round, over mapped pipe, and 1.9 m in
Nav2's backup recovery (8 times 0.3 m when the controller got stuck, backing
over the pipe just driven). It turned on the spot in a manhole 4 times. Of
the 8 frontiers left, 6 are gaps in the walls of pipes already driven
through, 1 lies next to the path driven, and 1 could not be reached. The
last whole-network run before these changes (16 Oct) took 1409 s, drove
404.8 m and left 24 frontiers; that is one run each, so part of the
difference is the run-to-run variation of Gazebo.

## Not done (yet)

From the research list: active SLAM and degeneracy-aware estimation (C1,
C3), worth it only if the map still drifts; GBPlanner, next-best-view and
fast frontier detection (B3 to B5); a planner plugin that forbids reversing
into unknown cells inside the search itself (A3 proper: here the explorer
picks a planner that can't enter unknown space instead).
