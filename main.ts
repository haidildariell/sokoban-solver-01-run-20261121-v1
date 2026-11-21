// Sokoban solver for make.fun sokoban-solver-01-run-20261121.
//
// Strategy (written from scratch for this entry):
//   1. Split the level into independent rooms. A box pushed out of a room's
//      door into the 1-wide hallway can never be pushed back, so each room
//      is solved on its own and the push counts add up.
//   2. Solve every room with A* over pushes. A state is the sorted box
//      squares plus the player's reachable region (named by its lowest
//      square). The heuristic is the minimum-cost assignment of Manhattan
//      distances box -> goal, which is admissible (and consistent: one push
//      moves one box one square, so the assignment cost changes by at most
//      one), so the first solved state popped is push-optimal.
//   3. Deadlock pruning: a box may only be pushed onto a "live" square
//      (one from which a lone box can be pulled back to some goal), and a
//      push that creates a frozen 2x2 block of walls/boxes with a box off
//      its goal is rejected. Neither prunes a solvable state.
//   4. Pushes are turned into player moves with a shortest walk (BFS) to
//      each push square. Only the push count is scored, but the replay must
//      be legal, so every walk avoids walls and boxes.
//   5. Everything is budgeted by work (expanded states), never by the wall
//      clock, so two runs write identical bytes. No clock, no randomness,
//      no environment reads affect the output.
//
// MIT License. See LICENSE beside this file.

import { mkdirSync, readdirSync, readFileSync, writeFileSync } from 'node:fs';
import { join } from 'node:path';

const INPUT_DIR = process.env.MAKEFUN_INPUT_DIR ?? '/input';
const OUTPUT_DIR = process.env.MAKEFUN_OUTPUT_DIR ?? '/output';

// Work budgets (expanded states). Rooms are tiny (a 6x6 slot, 3-4 boxes),
// so a room that needs more than this is outside what the generator draws.
const ROOM_ASTAR_BUDGET = 600_000;
const ROOM_BFS_BUDGET = 400_000;
const WHOLE_ASTAR_BUDGET = 1_000_000;

const DX = [0, 0, -1, 1];
const DY = [-1, 1, 0, 0];
const STEP = ['u', 'd', 'l', 'r'];
const PUSH = ['U', 'D', 'L', 'R'];

interface Level {
  w: number;
  h: number;
  wall: Uint8Array; // 1 = wall
  goals: number[]; // sorted
  boxes: number[]; // sorted
  player: number;
}

function parseLevel(text: string): Level {
  const rows = text.split('\n').filter((r) => r.length > 0);
  const h = rows.length;
  const w = Math.max(...rows.map((r) => r.length));
  const wall = new Uint8Array(w * h).fill(1);
  const goals: number[] = [];
  const boxes: number[] = [];
  let player = -1;
  rows.forEach((row, y) => {
    for (let x = 0; x < row.length; x++) {
      const ch = row[x];
      const i = y * w + x;
      if (ch === '#') continue;
      wall[i] = 0;
      if (ch === '.' || ch === '*' || ch === '+') goals.push(i);
      if (ch === '$' || ch === '*') boxes.push(i);
      if (ch === '@' || ch === '+') player = i;
    }
  });
  goals.sort((a, b) => a - b);
  boxes.sort((a, b) => a - b);
  return { w, h, wall, goals, boxes, player };
}

/** Squares a lone box can be pulled back from onto some goal. */
function liveSquares(level: Level): Uint8Array {
  const { w, wall, goals } = level;
  const live = new Uint8Array(wall.length);
  const queue: number[] = [];
  for (const g of goals) {
    live[g] = 1;
    queue.push(g);
  }
  for (let qi = 0; qi < queue.length; qi++) {
    const x = queue[qi]!;
    for (let k = 0; k < 4; k++) {
      const d = DY[k]! * w + DX[k]!;
      const y = x - d; // square the box is pulled from
      if (!wall[y] && !wall[y - d] && !live[y]) {
        live[y] = 1;
        queue.push(y);
      }
    }
  }
  return live;
}

interface Room {
  boxes: number[];
  goals: number[];
  done: boolean;
}

/**
 * Split the level into rooms: cut the dead (non-live) floor squares of the
 * player's row (the hallway), then take the connected components that hold
 * boxes. Returns null when the level does not split cleanly.
 */
function splitRooms(level: Level): Room[] | null {
  const { w, wall, goals, boxes, player } = level;
  const live = liveSquares(level);
  const row = Math.floor(player / w);
  const cut = new Uint8Array(wall.length);
  for (let i = row * w; i < (row + 1) * w; i++) {
    if (!wall[i] && !live[i]) cut[i] = 1;
  }
  if (boxes.some((b) => cut[b])) return null;
  const part = new Int32Array(wall.length).fill(-1);
  const rooms: Room[] = [];
  for (let s = 0; s < wall.length; s++) {
    if (wall[s] || cut[s] || part[s] >= 0) continue;
    const squares: number[] = [s];
    part[s] = rooms.length;
    for (let i = 0; i < squares.length; i++) {
      const p = squares[i]!;
      for (let k = 0; k < 4; k++) {
        const q = p + DY[k]! * w + DX[k]!;
        if (!wall[q] && !cut[q] && part[q] < 0) {
          part[q] = rooms.length;
          squares.push(q);
        }
      }
    }
    rooms.push({ boxes: [], goals: [], done: false });
  }
  for (const b of boxes) rooms[part[b]!]!.boxes.push(b);
  for (const g of goals) {
    if (part[g]! < 0) return null;
    rooms[part[g]!]!.goals.push(g);
  }
  if (rooms.some((r) => r.boxes.length !== r.goals.length)) return null;
  return rooms.filter((r) => r.boxes.length > 0);
}

/** Flood-fill the player's reachable region; returns its lowest square. */
function regionLow(
  w: number,
  wall: Uint8Array,
  occupied: Uint8Array,
  from: number,
  seen: Int32Array,
  stamp: number,
  stack: Int32Array,
): number {
  let low = from;
  let n = 0;
  stack[n++] = from;
  seen[from] = stamp;
  while (n > 0) {
    const p = stack[--n]!;
    if (p < low) low = p;
    for (let k = 0; k < 4; k++) {
      const q = p + DY[k]! * w + DX[k]!;
      if (wall[q] === 0 && occupied[q] === 0 && seen[q] !== stamp) {
        seen[q] = stamp;
        stack[n++] = q;
      }
    }
  }
  return low;
}

/** True when the box just pushed to `t` is stuck in a frozen 2x2 block. */
function frozenBlock(
  w: number,
  wall: Uint8Array,
  occupied: Uint8Array,
  goal: Uint8Array,
  t: number,
): boolean {
  for (const a of [t, t - 1, t - w, t - w - 1]) {
    const b = a + 1;
    const c = a + w;
    const d = c + 1;
    if (!(wall[a] || occupied[a])) continue;
    if (!(wall[b] || occupied[b])) continue;
    if (!(wall[c] || occupied[c])) continue;
    if (!(wall[d] || occupied[d])) continue;
    if ((occupied[a] && !goal[a]) || (occupied[b] && !goal[b]) ||
        (occupied[c] && !goal[c]) || (occupied[d] && !goal[d])) {
      return true;
    }
  }
  return false;
}

/** All permutations of 0..n-1 (n <= 4 here), for the assignment heuristic. */
function permutations(n: number): number[][] {
  const out: number[][] = [];
  const cur: number[] = [];
  const used = new Array<boolean>(n).fill(false);
  const rec = (): void => {
    if (cur.length === n) {
      out.push(cur.slice());
      return;
    }
    for (let i = 0; i < n; i++) {
      if (used[i]) continue;
      used[i] = true;
      cur.push(i);
      rec();
      cur.pop();
      used[i] = false;
    }
  };
  rec();
  return out;
}

interface PushStep {
  from: number;
  to: number;
  dir: number;
}

/**
 * Push-optimal solve of one (sub-)level with A* over pushes.
 * Returns the push list (box from -> to, in order) or null when the budget
 * runs out / no solution. `maxStates` is the expansion budget.
 */
function solveAStar(level: Level, maxStates: number): PushStep[] | null {
  const { w, wall, goals, boxes } = level;
  const size = wall.length;
  const n = boxes.length;
  const goal = new Uint8Array(size);
  for (const g of goals) goal[g] = 1;
  const live = liveSquares(level);
  if (boxes.some((b) => !live[b])) return null;

  const gx = goals.map((g) => g % w);
  const gy = goals.map((g) => Math.floor(g / w));
  const perms = permutations(n);
  // h(boxes): cheapest assignment of Manhattan distances box -> goal.
  const heuristic = (bs: number[]): number => {
    let best = Infinity;
    for (const p of perms) {
      let s = 0;
      for (let i = 0; i < n; i++) {
        const b = bs[i]!;
        s += Math.abs((b % w) - gx[p[i]!]!) + Math.abs(Math.floor(b / w) - gy[p[i]!]!);
      }
      if (s < best) best = s;
    }
    return best;
  };

  const solved = (bs: number[]): boolean => bs.every((b) => goal[b]);
  const occupied = new Uint8Array(size);
  const seen = new Int32Array(size);
  const mark = new Int32Array(size);
  const stack = new Int32Array(size);
  let stamp = 0;

  // State storage: boxes, player square after the push, parent, push step.
  const stBoxes: number[][] = [boxes.slice()];
  const stPlayer: number[] = [level.player];
  const stParent: number[] = [-1];
  const stPush: (PushStep | null)[] = [null];
  const stG: number[] = [0];
  // Key: region-low followed by the sorted box squares, base `size`.
  const keyOf = (bs: number[], low: number): number => {
    let k = low;
    for (const b of bs) k = k * size + b;
    return k;
  };
  const bestG = new Map<number, number>();
  for (const b of boxes) occupied[b] = 1;
  bestG.set(keyOf(boxes, regionLow(w, wall, occupied, level.player, seen, ++stamp, stack)), 0);
  for (const b of boxes) occupied[b] = 0;

  // Binary heap of [f, g, seq, state], ordered by f, then larger g, then seq.
  const heap: [number, number, number, number][] = [];
  let seq = 0;
  const heapLess = (a: [number, number, number, number], b: [number, number, number, number]): boolean =>
    a[0] < b[0] || (a[0] === b[0] && (a[1] > b[1] || (a[1] === b[1] && a[2] < b[2])));
  const heapPush = (e: [number, number, number, number]): void => {
    let i = heap.length;
    heap.push(e);
    while (i > 0) {
      const p = (i - 1) >> 1;
      if (!heapLess(heap[i]!, heap[p]!)) break;
      [heap[i], heap[p]] = [heap[p]!, heap[i]!];
      i = p;
    }
  };
  const heapPop = (): [number, number, number, number] => {
    const top = heap[0]!;
    const last = heap.pop()!;
    if (heap.length > 0) {
      heap[0] = last;
      for (let i = 0; ; ) {
        let c = 2 * i + 1;
        if (c >= heap.length) break;
        if (c + 1 < heap.length && heapLess(heap[c + 1]!, heap[c]!)) c++;
        if (!heapLess(heap[c]!, heap[i]!)) break;
        [heap[i], heap[c]] = [heap[c]!, heap[i]!];
        i = c;
      }
    }
    return top;
  };

  heapPush([heuristic(boxes), 0, seq++, 0]);
  let expanded = 0;
  let found = solved(boxes) ? 0 : -1;
  while (heap.length > 0 && found < 0) {
    const [, g, , s] = heapPop();
    if (bestG.get(keyOf(stBoxes[s]!, regionLow(w, wall, (() => {
      for (const b of stBoxes[s]!) occupied[b] = 1;
      return occupied;
    })(), stPlayer[s]!, seen, ++stamp, stack))) !== g) {
      // Stale entry: a cheaper path to this state was recorded since.
      for (const b of stBoxes[s]!) occupied[b] = 0;
      continue;
    }
    for (const b of stBoxes[s]!) occupied[b] = 0;
    if (expanded >= maxStates) return null;
    expanded++;
    const bs = stBoxes[s]!;
    for (const b of bs) occupied[b] = 1;
    const here = ++stamp;
    regionLow(w, wall, occupied, stPlayer[s]!, seen, here, stack);
    for (let bi = 0; bi < n && found < 0; bi++) {
      const box = bs[bi]!;
      for (let k = 0; k < 4; k++) {
        const d = DY[k]! * w + DX[k]!;
        const t = box + d;
        // The player must be able to stand behind the box.
        if (seen[box - d] !== here) continue;
        if (wall[t] || occupied[t] || !live[t]) continue;
        occupied[box] = 0;
        occupied[t] = 1;
        const frozen = frozenBlock(w, wall, occupied, goal, t);
        if (!frozen) {
          const next = bs.slice();
          next[bi] = t;
          next.sort((a, b) => a - b);
          const low = regionLow(w, wall, occupied, box, mark, ++stamp, stack);
          const key = keyOf(next, low);
          const ng = g + 1;
          const prevG = bestG.get(key);
          if (prevG === undefined || ng < prevG) {
            bestG.set(key, ng);
            const idx = stBoxes.length;
            stBoxes.push(next);
            stPlayer.push(box);
            stParent.push(s);
            stPush.push({ from: box, to: t, dir: k });
            stG.push(ng);
            if (solved(next)) {
              found = idx;
              break;
            }
            heapPush([ng + heuristic(next), ng, seq++, idx]);
          }
        }
        occupied[t] = 0;
        occupied[box] = 1;
      }
    }
    for (const b of bs) occupied[b] = 0;
  }
  if (found < 0) return null;
  const steps: PushStep[] = [];
  for (let s = found; s > 0; s = stParent[s]!) steps.push(stPush[s]!);
  steps.reverse();
  return steps;
}

/** Plain push-BFS fallback (same pruning, FIFO order), for stubborn rooms. */
function solveBfs(level: Level, maxStates: number): PushStep[] | null {
  const { w, wall, goals, boxes } = level;
  const size = wall.length;
  const goal = new Uint8Array(size);
  for (const g of goals) goal[g] = 1;
  const live = liveSquares(level);
  if (boxes.some((b) => !live[b])) return null;
  const solved = (bs: number[]): boolean => bs.every((b) => goal[b]);
  const occupied = new Uint8Array(size);
  const seen = new Int32Array(size);
  const mark = new Int32Array(size);
  const stack = new Int32Array(size);
  let stamp = 0;
  const stBoxes: number[][] = [boxes.slice()];
  const stPlayer: number[] = [level.player];
  const stParent: number[] = [-1];
  const stPush: (PushStep | null)[] = [null];
  const keyOf = (bs: number[], low: number): number => {
    let k = low;
    for (const b of bs) k = k * size + b;
    return k;
  };
  const index = new Map<number, number>();
  for (const b of boxes) occupied[b] = 1;
  index.set(keyOf(boxes, regionLow(w, wall, occupied, level.player, seen, ++stamp, stack)), 0);
  for (const b of boxes) occupied[b] = 0;
  let found = solved(boxes) ? 0 : -1;
  for (let head = 0; found < 0 && head < stBoxes.length; head++) {
    const bs = stBoxes[head]!;
    for (const b of bs) occupied[b] = 1;
    const here = ++stamp;
    regionLow(w, wall, occupied, stPlayer[head]!, seen, here, stack);
    for (let bi = 0; bi < bs.length && found < 0; bi++) {
      const box = bs[bi]!;
      for (let k = 0; k < 4; k++) {
        const d = DY[k]! * w + DX[k]!;
        const t = box + d;
        if (seen[box - d] !== here) continue;
        if (wall[t] || occupied[t] || !live[t]) continue;
        occupied[box] = 0;
        occupied[t] = 1;
        const frozen = frozenBlock(w, wall, occupied, goal, t);
        let key = -1;
        let next: number[] | null = null;
        if (!frozen) {
          next = bs.slice();
          next[bi] = t;
          next.sort((a, b) => a - b);
          key = keyOf(next, regionLow(w, wall, occupied, box, mark, ++stamp, stack));
        }
        occupied[t] = 0;
        occupied[box] = 1;
        if (frozen || index.has(key)) continue;
        if (stBoxes.length >= maxStates) return null;
        index.set(key, stBoxes.length);
        stBoxes.push(next!);
        stPlayer.push(box);
        stParent.push(head);
        stPush.push({ from: box, to: t, dir: k });
        if (solved(next!)) {
          found = stBoxes.length - 1;
          break;
        }
      }
    }
    for (const b of bs) occupied[b] = 0;
  }
  if (found < 0) return null;
  const steps: PushStep[] = [];
  for (let s = found; s > 0; s = stParent[s]!) steps.push(stPush[s]!);
  steps.reverse();
  return steps;
}

/** Shortest walk from `from` to `to` avoiding walls and boxes (lowercase). */
function walkTo(
  level: Level,
  occupied: Uint8Array,
  from: number,
  to: number,
): string {
  const { w, wall } = level;
  if (from === to) return '';
  const prev = new Int32Array(wall.length).fill(-1);
  const how = new Array<string>(wall.length);
  const queue: number[] = [from];
  prev[from] = from;
  for (let qi = 0; qi < queue.length; qi++) {
    const p = queue[qi]!;
    if (p === to) break;
    for (let k = 0; k < 4; k++) {
      const q = p + DY[k]! * w + DX[k]!;
      if (wall[q] || occupied[q] || prev[q] !== -1) continue;
      prev[q] = p;
      how[q] = STEP[k]!;
      queue.push(q);
    }
  }
  if (prev[to] === -1) throw new Error('walkTo: push square out of reach');
  let s = '';
  for (let p = to; p !== from; p = prev[p]!) s = how[p]! + s;
  return s;
}

/** Turn a push list into the player's move string on this level. */
function pushesToMoves(level: Level, steps: PushStep[]): string {
  const { wall } = level;
  const occupied = new Uint8Array(wall.length);
  for (const b of level.boxes) occupied[b] = 1;
  let moves = '';
  let player = level.player;
  const boxes = level.boxes.slice();
  for (const step of steps) {
    const d = DY[step.dir]! * level.w + DX[step.dir]!;
    moves += walkTo(level, occupied, player, step.from - d);
    moves += PUSH[step.dir]!;
    occupied[step.from] = 0;
    occupied[step.to] = 1;
    boxes[boxes.indexOf(step.from)] = step.to;
    boxes.sort((a, b) => a - b);
    player = step.from;
  }
  return moves;
}

function solveLevel(level: Level): string {
  const rooms = splitRooms(level);
  if (rooms !== null) {
    let player = level.player;
    let moves = '';
    for (const room of rooms) {
      // Other rooms' boxes act as walls (on their goals once solved).
      const wall = level.wall.slice();
      for (const other of rooms) {
        if (other === room) continue;
        for (const b of other.done ? other.goals : other.boxes) wall[b] = 1;
      }
      const sub: Level = {
        w: level.w,
        h: level.h,
        wall,
        goals: room.goals.slice().sort((a, b) => a - b),
        boxes: room.boxes.slice().sort((a, b) => a - b),
        player,
      };
      let steps = solveAStar(sub, ROOM_ASTAR_BUDGET);
      if (steps === null) steps = solveBfs(sub, ROOM_BFS_BUDGET);
      if (steps === null) return '';
      const roomMoves = pushesToMoves(sub, steps);
      moves += roomMoves;
      // Advance the player to where this room's moves left it.
      for (const ch of roomMoves.toLowerCase()) {
        player += ch === 'u' ? -level.w : ch === 'd' ? level.w : ch === 'l' ? -1 : 1;
      }
      room.done = true;
    }
    return moves;
  }
  // No clean room split: solve the whole level as one problem.
  let steps = solveAStar(level, WHOLE_ASTAR_BUDGET);
  if (steps === null) steps = solveBfs(level, ROOM_BFS_BUDGET);
  if (steps === null) return '';
  return pushesToMoves(level, steps);
}

for (const id of readdirSync(INPUT_DIR).sort()) {
  const moves = solveLevel(parseLevel(readFileSync(join(INPUT_DIR, id, 'level.txt'), 'utf8')));
  mkdirSync(join(OUTPUT_DIR, id), { recursive: true });
  writeFileSync(join(OUTPUT_DIR, id, 'solution.txt'), `${moves}\n`);
}
