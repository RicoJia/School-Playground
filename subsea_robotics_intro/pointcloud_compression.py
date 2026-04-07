import numpy as np
import zstandard as zstd
import struct
import sqlite3
import argparse
from pathlib import Path
from typing import Tuple



import numpy as np
import struct
import zstandard as zstd
from collections import deque

MAGIC = b"GPCC"  # 4 bytes

def _encode_octree(voxels: np.ndarray, qbits: int) -> bytes:
    """
    voxels: (M,3) int32 in [0, 2^qbits)
    Returns raw occupancy bytes (one byte per internal node) in BFS order.
    """
    voxels = np.asarray(voxels, dtype=np.int32)
    M = voxels.shape[0]
    if M == 0:
        return b""

    occupancy = bytearray()
    # Each node: (indices into voxels, depth)
    queue = deque()
    all_indices = np.arange(M, dtype=np.int64)
    queue.append((all_indices, 0))

    while queue:
        idxs, depth = queue.popleft()
        if depth == qbits:
            # Leaf level: one voxel per node; no occupancy byte
            continue

        pts = voxels[idxs]  # (k,3)
        bitpos = qbits - 1 - depth

        xbits = (pts[:, 0] >> bitpos) & 1
        ybits = (pts[:, 1] >> bitpos) & 1
        zbits = (pts[:, 2] >> bitpos) & 1
        child_ids = (xbits << 2) | (ybits << 1) | zbits  # 0..7

        occ = 0
        for child_id in range(8):
            mask = (child_ids == child_id)
            if not np.any(mask):
                continue
            occ |= (1 << child_id)
        occupancy.append(occ)

        # Enqueue children in fixed order 0..7 to match decoder
        for child_id in range(8):
            mask = (child_ids == child_id)
            if not np.any(mask):
                continue
            child_indices = idxs[mask]
            queue.append((child_indices, depth + 1))

    return bytes(occupancy)

def _decode_octree(occupancy_bytes: bytes, qbits: int) -> np.ndarray:
    """
    occupancy_bytes: raw occupancy stream in BFS order
    Returns: (M,3) int32 voxel coordinates in [0, 2^qbits)
    """
    if qbits == 0:
        return np.zeros((0, 3), dtype=np.int32)

    occupancy = list(occupancy_bytes)
    occ_idx = 0

    voxels = []
    queue = deque()
    # Node = (base_x, base_y, base_z, depth)
    queue.append((0, 0, 0, 0))

    while queue:
        base_x, base_y, base_z, depth = queue.popleft()

        if depth == qbits:
            # Leaf voxel: cube of size 1 at base_x,y,z
            voxels.append((base_x, base_y, base_z))
            continue

        if occ_idx >= len(occupancy):
            raise ValueError("Ran out of occupancy bytes while decoding")
        occ = occupancy[occ_idx]
        occ_idx += 1

        step = 1 << (qbits - depth - 1)  # size of child cell along each axis

        for child_id in range(8):
            if not (occ & (1 << child_id)):
                continue
            xbit = (child_id >> 2) & 1
            ybit = (child_id >> 1) & 1
            zbit = child_id & 1

            child_base_x = base_x + xbit * step
            child_base_y = base_y + ybit * step
            child_base_z = base_z + zbit * step

            queue.append((child_base_x, child_base_y, child_base_z, depth + 1))

    # Optionally: check that we've consumed all occupancy bytes
    if occ_idx != len(occupancy):
        # Extra bytes => bitstream mismatch; you could raise or ignore
        pass

    return np.asarray(voxels, dtype=np.int32)


def compress_point_cloud(points: np.ndarray,
                              qbits: int = 8,
                              zstd_level: int = 10) -> bytes:
    """
    G-PCC-style geometry compressor (simplified):
    - Quantize to [0, 2^qbits - 1]^3
    - Deduplicate voxels
    - Build full-depth octree, emit occupancy bytes in BFS order
    - zstd-compress [header + occupancy_bytes]

    Header format (little-endian):
      4s   magic: b"GPCC"
      B    qbits
      3x   padding
      6f   mins[3], maxs[3]
    """
    points = np.asarray(points, dtype=np.float32)
    assert points.ndim == 2 and points.shape[1] == 3, "points must be (N,3)"

    n = points.shape[0]
    if n == 0:
        # empty cloud: encode header with dummy bbox and no occupancy
        mins = np.zeros(3, dtype=np.float32)
        maxs = np.zeros(3, dtype=np.float32)
        header = struct.pack("<4sB3x6f", MAGIC, qbits, *(mins.tolist() + maxs.tolist()))
        cctx = zstd.ZstdCompressor(level=zstd_level)
        return cctx.compress(header)

    # Bounding box
    mins = points.min(axis=0)
    maxs = points.max(axis=0)
    ranges = np.maximum(maxs - mins, 1e-12)

    # Quantize to integer grid
    max_val = (1 << qbits) - 1
    norm = (points - mins) / ranges
    q = np.clip(np.round(norm * max_val), 0, max_val).astype(np.int32)

    # Deduplicate voxels (G-PCC typically operates on occupied voxels)
    q_unique = np.unique(q, axis=0)

    # Octree occupancy coding
    occupancy_bytes = _encode_octree(q_unique, qbits)

    # Header
    header = struct.pack("<4sB3x6f", MAGIC, qbits, *(mins.tolist() + maxs.tolist()))

    # Concatenate header + occupancy
    raw = header + occupancy_bytes

    # Compress with zstd
    cctx = zstd.ZstdCompressor(level=zstd_level)
    return cctx.compress(raw)


def decompress_point_cloud(data: bytes) -> np.ndarray:
    """
    Decompress G-PCC-style geometry:
    - zstd decompress
    - parse header
    - decode octree occupancy to reconstruct quantized voxels
    - dequantize back to float positions (voxel centers)
    """
    dctx = zstd.ZstdDecompressor()
    raw = dctx.decompress(data)

    header_size = struct.calcsize("<4sB3x6f")
    if len(raw) < header_size:
        raise ValueError("Data too short for GPCC header")

    magic, qbits, *bbox = struct.unpack("<4sB3x6f", raw[:header_size])
    if magic != MAGIC:
        raise ValueError("Not a GPCC-style stream (bad magic)")

    mins = np.array(bbox[0:3], dtype=np.float32)
    maxs = np.array(bbox[3:6], dtype=np.float32)
    ranges = np.maximum(maxs - mins, 1e-12)

    occupancy_bytes = raw[header_size:]

    if len(occupancy_bytes) == 0:
        return np.zeros((0, 3), dtype=np.float32)

    # Reconstruct integer voxels
    voxels = _decode_octree(occupancy_bytes, qbits)  # (M,3) int32 in [0, 2^qbits)

    max_val = (1 << qbits) - 1
    voxels_f = voxels.astype(np.float32)

    # Dequantize (map 0..max_val -> bbox)
    norm = voxels_f / max_val
    pts = mins + norm * ranges

    return pts



def filter_by_range(point_cloud: np.ndarray, min_range: float = 0.5, max_range: float = 20.0) -> np.ndarray:
    """Filter point cloud by distance from origin."""
    distances = np.linalg.norm(point_cloud, axis=1)
    mask = (distances >= min_range) & (distances <= max_range)
    return point_cloud[mask]

def voxel_downsample(point_cloud: np.ndarray, leaf_size: float = 0.03) -> np.ndarray:
    """Downsample point cloud using voxel grid filter."""
    if len(point_cloud) == 0:
        return point_cloud
    
    # Compute voxel indices for each point
    voxel_indices = np.floor(point_cloud / leaf_size).astype(np.int32)
    
    # Create unique voxel identifier
    # Use a hash-like approach for unique voxel IDs
    min_idx = voxel_indices.min(axis=0)
    voxel_indices_shifted = voxel_indices - min_idx
    
    # Get unique voxels and their indices
    _, unique_indices = np.unique(
        voxel_indices_shifted[:, 0] * 1000000 + 
        voxel_indices_shifted[:, 1] * 1000 + 
        voxel_indices_shifted[:, 2],
        return_index=True
    )
    
    return point_cloud[unique_indices]
