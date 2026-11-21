# Sokoban Solver — make.fun sokoban-solver-01-run-20261121

Push-optimal Sokoban solver for the make.fun competition
"Build the best Sokoban solver, 21 November 2026".
One program (`main.ts`, Node 22, no dependencies) answers 5 instances
in one run, writing `/output/<id>/solution.txt` for each.

## How it works

1. **Room split.** A box pushed out of a room's door into the 1-wide
   hallway can never be pushed back, so each room is an independent
   problem. The level is split by cutting the dead (non-live) floor
   squares of the player's row (the hallway) and taking the connected
   components that hold boxes.
2. **Per-room A\* over pushes.** A state is the sorted box squares plus
   the player's reachable region (named by its lowest square). The
   heuristic is the minimum-cost assignment of Manhattan distances
   box → goal, which is admissible and consistent, so the first solved
   state popped is push-optimal.
3. **Deadlock pruning.** A box may only be pushed onto a *live* square
   (one from which a lone box can be pulled back to some goal), and a
   push that creates a frozen 2×2 block of walls/boxes with a box off
   its goal is rejected. Neither prunes a solvable state.
4. **Move reconstruction.** Pushes become player moves with a shortest
   walk (BFS) to each push square. Only the push count is scored, but
   the replay must be legal, so every walk avoids walls and boxes.
5. **Determinism.** All budgets are work (expanded states), never the
   wall clock. No clock, no randomness, no environment reads affect the
   output — two runs write identical bytes. A plain push-BFS with the
   same pruning is the fallback if A\* exhausts its budget, and a
   whole-level search is the fallback if a level does not split cleanly.

## Results (kit scorer, local)

- Practice set (5 levels): **1,000,000 / 1,000,000** — every instance
  solved in exactly P\* pushes (13, 16, 29, 47, 60). Runtime ~0.4 s.
- 25 extra levels drawn with the kit generator (5 fresh sets):
  **1,000,000 on every set**, ~0.4 s per set.

## Run locally

```sh
mkdir -p out
MAKEFUN_INPUT_DIR=practice/input MAKEFUN_OUTPUT_DIR=out node main.ts
```

## License

MIT — see LICENSE.
