
#!/usr/bin/env python3

from typing import List, Optional, Tuple, Dict, Set

WORD_TO_LETTER = {
    "red": "R", "orange": "O", "green": "G", "blue": "B", "purple": "P", "white": ".",
    "r": "R", "o": "O", "g": "G", "b": "B", "p": "P", ".": ".", "?": ".",
}

# Block definitions (order-insensitive)
BLOCKS = {
    1: ["RR", "PG", "BO", "OG"],
    2: ["PP", "GB", "OR", "BO"],
    3: ["GG", "RP", "PB", "OR"],
    4: ["BB", "GP", "RG", "PO"],
    5: ["OO", "RB", "BG", "PR"],
}

def _norm_pair(a: str, b: str) -> str:
    # order-insensitive pair, preserves doubles (RR)
    return "".join(sorted([a, b]))


# Precompute allowed pairs per block
block_pairs: Dict[int, Set[str]] = {
    k: {_norm_pair(p[0], p[1]) for p in pairs}
    for k, pairs in BLOCKS.items()
}

def solve(labels: List[List[str]]) -> Optional[List[List[int]]]:
    """
    labels: 3x4, each entry can be "red"/"green"/... or "R"/"G"/... or "." for empty.
    returns: 3x4 ints, 0 for empty, else block number 1..5.
             returns None if no solution found.
    """
    H, W = 3, 4

    # Normalize board to letters -> ][[O, R, G, B, P, .], [....]]
    board = []
    for r in range(H):
        row = []
        for c in range(W):
            v = labels[r][c]
            if v is None:
                row.append(".")
            else:
                key = str(v).strip().lower()
                row.append(WORD_TO_LETTER.get(key, WORD_TO_LETTER.get(str(v).strip(), ".")))
        board.append(row)

    # Cells to cover (non-empty)
    cells = [(r, c) for r in range(H) for c in range(W) if board[r][c] != "."]

    # Each placement = (block_id, (r1,c1), (r2,c2))
    placements = []
    for k in BLOCKS.keys():
        for r in range(H):
            for c in range(W):
                if board[r][c] == ".":
                    continue
                # right neighbor
                if c + 1 < W and board[r][c+1] != ".":
                    pair = _norm_pair(board[r][c], board[r][c+1])
                    if pair in block_pairs[k]:
                        placements.append((k, (r, c), (r, c+1)))
                # down neighbor
                if r + 1 < H and board[r+1][c] != ".":
                    pair = _norm_pair(board[r][c], board[r+1][c])
                    if pair in block_pairs[k]:
                        placements.append((k, (r, c), (r+1, c)))
    # Index placements by cell (for fast branching)
    by_cell: Dict[Tuple[int, int], List[Tuple[int, Tuple[int,int], Tuple[int,int]]]] = {cell: [] for cell in cells}
    for pl in placements:
        _, a, b = pl
        if a in by_cell: by_cell[a].append(pl)
        if b in by_cell: by_cell[b].append(pl)

    # Backtracking state
    used_blocks = set()
    covered = set()
    assignment = [[0 for _ in range(W)] for _ in range(H)]
    def pick_next_cell() -> Optional[Tuple[int, int]]:
        # choose uncovered cell with fewest candidate placements (MRV heuristic)
        best = None
        best_count = 10**9
        for cell in cells:
            if cell in covered:
                continue
            opts = [pl for pl in by_cell[cell] if pl[0] not in used_blocks
                    and pl[1] not in covered and pl[2] not in covered]
            cnt = len(opts)
            if cnt == 0:
                return cell  # dead end quickly
            if cnt < best_count:
                best_count = cnt
                best = cell
        return best

    def dfs() -> bool:
        if len(covered) == len(cells):
            # If you want "must use all 5 blocks", uncomment:
            # return len(used_blocks) == 5
            return True

        cell = pick_next_cell()
        if cell is None:
            return False

        # feasible placements covering this cell
        options = [pl for pl in by_cell[cell] if pl[0] not in used_blocks
                   and pl[1] not in covered and pl[2] not in covered]
        if not options:
            return False

        for k, a, b in options:
            # place block k covering a and b
            used_blocks.add(k)
            covered.add(a); covered.add(b)
            assignment[a[0]][a[1]] = k
            assignment[b[0]][b[1]] = k

            if dfs():
                return True

            # undo
            assignment[a[0]][a[1]] = 0
            assignment[b[0]][b[1]] = 0
            covered.remove(a); covered.remove(b)
            used_blocks.remove(k)

        return False

    ok = dfs()
    return assignment if ok else None

