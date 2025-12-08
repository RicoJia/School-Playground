#!/usr/bin/env python3
r""" 
- using dpcc
    python3 test_pointcloud_compression.py --mode visualize --rosbag 2025-12-03_14-34-56_riser_run1 --compression dpcc --dpcc-model ./d-pcc/output/2024-10-24T10:30:23.340880/ckpt/ckpt-best.pth
- using octree
    python3 test_pointcloud_compression.py --mode visualize --rosbag 2025-12-03_14-34-56_riser_run1 --compression octree
"""

from pointcloud_compression import *

import numpy as np
import zstandard as zstd
import struct
import sqlite3
import argparse
from pathlib import Path
from typing import Tuple, Optional
import open3d as o3d
import time
import sys

from mcap.reader import make_reader
MCAP_AVAILABLE = True

# Check if d-pcc is available (but don't import yet to avoid argument parsing conflicts)
DPCC_PATH = Path(__file__).parent / "d-pcc"
DPCC_AVAILABLE = DPCC_PATH.exists()

def _lazy_import_dpcc():
    """Lazy import D-PCC modules to avoid argparse conflicts."""
    if not DPCC_AVAILABLE:
        return False
    
    try:
        sys.path.insert(0, str(DPCC_PATH))
        import torch
        from dpcc.models.autoencoder import AutoEncoder
        from dpcc.args.shapenet_args import parse_shapenet_args
        from dpcc.models.utils import AverageMeter
        print(f"D-PCC module loaded from {DPCC_PATH}")
        return True, torch, AutoEncoder, parse_shapenet_args, AverageMeter
    except ImportError as e:
        print(f"Warning: Could not import D-PCC modules: {e}")
        return False

class DPCCCompressor:
    """Wrapper for D-PCC neural compression."""
    
    def __init__(self, model_path: Optional[str] = None, downsample_rate: float = 0.5, max_upsample: int = 16):
        """Initialize D-PCC compressor.
        
        Args:
            model_path: Path to model checkpoint
            downsample_rate: Rate to keep points at each layer (0.33-0.67, higher = less lossy)
            max_upsample: Max points to generate during upsampling (8-32, higher = less lossy)
        """
        # Lazy import D-PCC to avoid argparse conflicts
        imports = _lazy_import_dpcc()
        if not imports:
            raise RuntimeError("D-PCC not available. Check installation and path.")
        
        _, torch, AutoEncoder, parse_shapenet_args, AverageMeter = imports
        self.torch = torch
        self.AutoEncoder = AutoEncoder
        
        self.device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
        print(f"Using device: {self.device}")
        
        # Parse default args (pass empty list to avoid parsing sys.argv)
        import sys as _sys
        old_argv = _sys.argv
        try:
            _sys.argv = [_sys.argv[0]]  # Keep only script name
            self.args = parse_shapenet_args()
            # Override hidden_dim to match checkpoint architecture
            self.args.hidden_dim = 128
        except Exception as e:
            print(f"Warning: Error parsing D-PCC args: {e}, using defaults")
            # Create a minimal args object
            class Args:
                pass
            self.args = Args()
            self.args.quantize_latent_xyzs = True
            self.args.latent_xyzs_conv_mode = 'mlp'
            self.args.sub_point_conv_mode = 'mlp'
            self.args.in_fdim = 3
            self.args.dim = 8
            self.args.hidden_dim = 128  # Match checkpoint architecture
            self.args.ngroups = 1
            self.args.k = 16
            self.args.layer_num = 3
            self.args.downsample_rate = [downsample_rate] * 3
            self.args.max_upsample_num = [max_upsample] * 3
        finally:
            _sys.argv = old_argv  # Restore original argv
        
        # The default has mismatched layer_num vs list lengths
        # Adjust to make lists match layer_num or vice versa
        if hasattr(self.args, 'downsample_rate') and hasattr(self.args, 'layer_num'):
            list_len = len(self.args.downsample_rate)
            if self.args.layer_num != list_len:
                print(f"Adjusting layer_num from {self.args.layer_num} to {list_len} to match downsample_rate length")
                self.args.layer_num = list_len
        
        if model_path and Path(model_path).exists():
            print(f"Creating AutoEncoder model...")
            print(f"  layer_num: {self.args.layer_num}")
            print(f"  downsample_rate: {self.args.downsample_rate}")
            print(f"  max_upsample_num: {self.args.max_upsample_num}")
            self.model = AutoEncoder(self.args).to(self.device)
            print(f"Loading model weights from {model_path}...")
            # Load with strict=False to ignore incompatible entropy bottleneck layers
            # (these will be re-initialized by update() anyway)
            self.model.load_state_dict(torch.load(model_path, map_location=self.device), strict=False)
            print(f"Updating entropy bottlenecks...")
            self.model.feats_eblock.update(force=True)
            if self.args.quantize_latent_xyzs:
                self.model.xyzs_eblock.update(force=True)
            self.model.eval()
            self.model_loaded = True
            print(f"D-PCC model loaded successfully from {model_path}")
        else:
            self.model = None
            self.model_loaded = False
            if model_path:
                print(f"Warning: Model path {model_path} not found. D-PCC will not be functional.")
    
    def compress(self, points: np.ndarray) -> bytes:
        """Compress point cloud using D-PCC."""
        if not self.model_loaded:
            raise RuntimeError("D-PCC model not loaded")
        
        # Convert to torch tensor: (1, 3, N)
        xyzs = self.torch.from_numpy(points.T).float().unsqueeze(0).to(self.device)
        feats = xyzs.clone()
        
        with self.torch.no_grad():
            # Raise dimension
            feats = self.model.pre_conv(feats)
            
            # Encoder forward
            gt_xyzs, gt_dnums, gt_mdis, latent_xyzs, latent_feats = self.model.encoder(xyzs, feats)
            
            # Compress latent features
            feats_size = latent_feats.size()[2:]
            latent_feats_str = self.model.feats_eblock.compress(latent_feats)
            
            # Compress latent xyzs
            if self.args.quantize_latent_xyzs:
                analyzed_latent_xyzs = self.model.latent_xyzs_analysis(latent_xyzs)
                xyzs_size = analyzed_latent_xyzs.size()[2:]
                latent_xyzs_str = self.model.xyzs_eblock.compress(analyzed_latent_xyzs)
            else:
                # Half float representation
                latent_xyzs_str = latent_xyzs.half()
                xyzs_size = None
        
        # Package everything for decompression
        import pickle
        compressed_data = pickle.dumps({
            'latent_xyzs_str': latent_xyzs_str,
            'xyzs_size': xyzs_size,
            'latent_feats_str': latent_feats_str,
            'feats_size': feats_size,
            'quantize_latent_xyzs': self.args.quantize_latent_xyzs
        })
        
        return compressed_data
    
    def decompress(self, data: bytes) -> np.ndarray:
        """Decompress point cloud using D-PCC."""
        if not self.model_loaded:
            raise RuntimeError("D-PCC model not loaded")
        
        import pickle
        compressed_dict = pickle.loads(data)
        
        with self.torch.no_grad():
            # Decompress latent xyzs
            if compressed_dict['quantize_latent_xyzs']:
                analyzed_latent_xyzs_hat = self.model.xyzs_eblock.decompress(
                    compressed_dict['latent_xyzs_str'], 
                    compressed_dict['xyzs_size']
                )
                latent_xyzs_hat = self.model.latent_xyzs_synthesis(analyzed_latent_xyzs_hat)
            else:
                latent_xyzs_hat = compressed_dict['latent_xyzs_str']
            
            # Decompress latent features
            latent_feats_hat = self.model.feats_eblock.decompress(
                compressed_dict['latent_feats_str'], 
                compressed_dict['feats_size']
            )
            
            # Decoder forward
            pred_xyzs, pred_unums, pred_mdis, upsampled_feats = self.model.decoder(
                latent_xyzs_hat, 
                latent_feats_hat
            )
        
        # Convert back to numpy: (N, 3)
        points = pred_xyzs[-1].squeeze(0).permute(1, 0).cpu().numpy()
        return points


def parse_pointcloud2_msg(data: bytes) -> np.ndarray:
    """Parse a ROS2 PointCloud2 message (CDR format) and extract XYZ points."""
    # Skip CDR header (4 bytes: encapsulation kind + options)
    offset = 4
    
    # Parse std_msgs/Header
    # stamp.sec (int32), stamp.nanosec (uint32)
    offset += 8
    
    # frame_id (string: uint32 length + chars + alignment)
    frame_id_len = struct.unpack_from('<I', data, offset)[0]
    offset += 4 + frame_id_len
    # Align to 4 bytes
    offset = (offset + 3) & ~3
    
    # uint32 height
    height = struct.unpack_from('<I', data, offset)[0]
    offset += 4
    
    # uint32 width
    width = struct.unpack_from('<I', data, offset)[0]
    offset += 4
    
    # sensor_msgs/PointField[] fields (sequence)
    fields_len = struct.unpack_from('<I', data, offset)[0]
    offset += 4
    
    # Skip field definitions
    for _ in range(fields_len):
        # name (string: uint32 length + chars)
        name_len = struct.unpack_from('<I', data, offset)[0]
        offset += 4 + name_len
        # Align to 4 bytes
        offset = (offset + 3) & ~3
        # offset (uint32)
        offset += 4
        # datatype (uint8)
        offset += 1
        # Align to 4 bytes
        offset = (offset + 3) & ~3
        # count (uint32)
        offset += 4
    
    # bool is_bigendian (uint8)
    offset += 1
    # Align to 4 bytes
    offset = (offset + 3) & ~3
    
    # uint32 point_step
    point_step = struct.unpack_from('<I', data, offset)[0]
    offset += 4
    
    # uint32 row_step
    row_step = struct.unpack_from('<I', data, offset)[0]
    offset += 4
    
    # uint8[] data (sequence: uint32 length + bytes)
    data_len = struct.unpack_from('<I', data, offset)[0]
    offset += 4
    
    # Extract point cloud data
    point_data = data[offset:offset + data_len]
    
    # Parse points (assuming float32 XYZ layout)
    num_points = height * width
    if num_points == 0 or point_step == 0:
        return np.array([]).reshape(0, 3)
    
    points = np.zeros((num_points, 3), dtype=np.float32)
    for i in range(num_points):
        start = i * point_step
        # Extract x, y, z (first 12 bytes as 3 float32s)
        points[i] = struct.unpack_from('<3f', point_data, start)
    
    return points

def find_bag_files(rosbag_path: str) -> Tuple[list, str]:
    """Find the .db3 or .mcap files in the rosbag directory."""
    bag_dir = Path(rosbag_path)
    
    # Try .db3 first
    db3_files = list(bag_dir.glob("*.db3"))
    if db3_files:
        return db3_files, 'db3'
    
    # Try .mcap
    mcap_files = sorted(bag_dir.glob("*.mcap"))
    return mcap_files, 'mcap'
    
    raise ValueError(f"No .db3 or .mcap files found in {rosbag_path}")

def load_first_lidar_msg(rosbag_path: str) -> np.ndarray:
    """Load the first lidar message from a ROS2 bag."""
    bag_files, bag_type = find_bag_files(rosbag_path)
    
    if bag_type == 'db3':
        conn = sqlite3.connect(str(bag_files[0]))
        cursor = conn.cursor()
        
        cursor.execute("""
            SELECT m.data 
            FROM messages m
            JOIN topics t ON m.topic_id = t.id
            WHERE t.name = '/wayfinder/head_multibeam_sonar/cloud'
            ORDER BY m.timestamp
            LIMIT 1
        """)
        
        row = cursor.fetchone()
        conn.close()
        
        if row is None:
            raise ValueError("No point cloud messages found in rosbag")
        
        return parse_pointcloud2_msg(row[0])
    
    else:  # mcap
        for mcap_file in bag_files:
            with open(mcap_file, 'rb') as f:
                reader = make_reader(f)
                for schema, channel, message in reader.iter_messages(topics=['/wayfinder/head_multibeam_sonar/cloud']):
                    return parse_pointcloud2_msg(message.data)
        
        raise ValueError("No point cloud messages found in rosbag")


def load_all_lidar_msgs(rosbag_path: str):
    """Generator that yields all lidar messages from a ROS2 bag."""
    bag_files, bag_type = find_bag_files(rosbag_path)
    
    if bag_type == 'db3':
        conn = sqlite3.connect(str(bag_files[0]))
        cursor = conn.cursor()
        
        cursor.execute("""
            SELECT m.data, m.timestamp
            FROM messages m
            JOIN topics t ON m.topic_id = t.id
            WHERE t.name = '/wayfinder/head_multibeam_sonar/cloud'
            ORDER BY m.timestamp
        """)
        
        for row in cursor:
            yield parse_pointcloud2_msg(row[0]), row[1]
        
        conn.close()
    
    else:  # mcap
        for mcap_file in bag_files:
            with open(mcap_file, 'rb') as f:
                reader = make_reader(f)
                for schema, channel, message in reader.iter_messages(topics=['/wayfinder/head_multibeam_sonar/cloud']):
                    yield parse_pointcloud2_msg(message.data), message.log_time


def process_and_visualize_bag(rosbag_path: str, min_range: float, max_range: float, leaf_size: float, 
                              compression_method: str = 'octree', dpcc_model_path: Optional[str] = None,
                              dpcc_downsample: float = 1.0, dpcc_upsample: int = 32):
    """Read all point clouds from bag, compress/decompress, and visualize."""
    
    # Initialize compressor
    dpcc_compressor = None
    if compression_method == 'dpcc':
        if not DPCC_AVAILABLE:
            raise RuntimeError("Error: D-PCC not available. Please check installation.")
        if not dpcc_model_path or not Path(dpcc_model_path).exists():
            raise RuntimeError(f"Error: D-PCC model path not found: {dpcc_model_path}")
        dpcc_compressor = DPCCCompressor(
            dpcc_model_path,
            downsample_rate=dpcc_downsample,
            max_upsample=dpcc_upsample
        )
        if not dpcc_compressor.model_loaded:
            raise RuntimeError("Error: D-PCC model failed to load.")
    
    # Initialize Open3D visualizer
    vis = o3d.visualization.Visualizer()
    vis.create_window(window_name=f"Point Cloud Compression ({compression_method.upper()})", width=1280, height=720)
    
    # Create point cloud object
    pcd = o3d.geometry.PointCloud()
    
    # Add geometry to visualizer
    vis.add_geometry(pcd)
    
    # Get render options
    opt = vis.get_render_option()
    opt.point_size = 5.0
    opt.background_color = np.array([0.05, 0.05, 0.05])
    
    # Statistics
    total_original_size = 0
    total_compressed_size = 0
    total_messages = 0
    total_compress_time = 0.0
    total_decompress_time = 0.0
    
    print(f"\nProcessing all lidar messages from {rosbag_path}")
    print(f"Compression method: {compression_method.upper()}")
    print(f"Range filter: {min_range}m - {max_range}m")
    print(f"Voxel leaf size: {leaf_size}m\n")
    print("-" * 80)
    
    try:
        for point_cloud, timestamp in load_all_lidar_msgs(str(rosbag_path)):
            total_messages += 1
            
            # Apply range filter
            filtered = filter_by_range(point_cloud, min_range, max_range)
            
            # Apply voxel downsampling
            downsampled = voxel_downsample(filtered, leaf_size)
            
            if downsampled.shape[0] == 0:
                print(f"Message {total_messages}: No points after filtering, skipping...")
                continue
            
            # Compress using selected method
            compress_start = time.time()
            if compression_method == 'dpcc':
                compressed = dpcc_compressor.compress(downsampled)
            else:  # octree
                compressed = compress_point_cloud(downsampled)
            compress_time = time.time() - compress_start
            
            # Decompress
            decompress_start = time.time()
            if compression_method == 'dpcc':
                decompressed = dpcc_compressor.decompress(compressed)
            else:  # octree
                decompressed = decompress_point_cloud(compressed)
            decompress_time = time.time() - decompress_start
            
            # Update statistics
            orig_size = downsampled.nbytes
            comp_size = len(compressed)
            total_original_size += orig_size
            total_compressed_size += comp_size
            total_compress_time += compress_time
            total_decompress_time += decompress_time
            ratio = orig_size / comp_size if comp_size > 0 else 0
            
            # Print stats for this message
            print(f"Msg {total_messages:3d}: {point_cloud.shape[0]:5d} pts → "
                  f"filter: {filtered.shape[0]:5d} → "
                  f"downsample: {downsampled.shape[0]:4d} → "
                  f"decomp: {decompressed.shape[0]:4d} | "
                  f"{orig_size:6d}B → {comp_size:4d}B ({ratio:.1f}x) | "
                  f"comp: {compress_time*1000:.1f}ms, decomp: {decompress_time*1000:.1f}ms")
            
            # Update visualization with decompressed point cloud
            pcd.points = o3d.utility.Vector3dVector(decompressed)
            
            # Color points by height (z-axis) with brighter colors
            if decompressed.shape[0] > 0:
                z_values = decompressed[:, 2]
                z_min, z_max = z_values.min(), z_values.max()
                if z_max > z_min:
                    colors = np.zeros((decompressed.shape[0], 3))
                    normalized_z = (z_values - z_min) / (z_max - z_min)
                    # Colormap: bright blue (low) to bright red (high) with cyan/yellow mix
                    colors[:, 0] = 0.3 + 0.7 * normalized_z  # Red (0.3 to 1.0)
                    colors[:, 1] = 0.3 * (1.0 - abs(normalized_z - 0.5) * 2)  # Green (peaks at middle)
                    colors[:, 2] = 0.3 + 0.7 * (1.0 - normalized_z)  # Blue (1.0 to 0.3)
                    pcd.colors = o3d.utility.Vector3dVector(colors)
            
            vis.update_geometry(pcd)
            
            # Reset view on first message
            if total_messages == 1:
                vis.reset_view_point(True)
            
            vis.poll_events()
            vis.update_renderer()
            
            # Small delay to see the visualization
            time.sleep(0.1)
            
            # Check if window is closed
            if not vis.poll_events():
                break
    
    except KeyboardInterrupt:
        print("\n\nInterrupted by user")
    
    finally:
        vis.destroy_window()
        
        # Print summary statistics
        print("\n" + "=" * 80)
        print("SUMMARY")
        print("=" * 80)
        print(f"Total messages processed: {total_messages}")
        print(f"Total original size: {total_original_size:,} bytes")
        print(f"Total compressed size: {total_compressed_size:,} bytes")
        if total_compressed_size > 0:
            overall_ratio = total_original_size / total_compressed_size
            print(f"Overall compression ratio: {overall_ratio:.2f}x")
            print(f"Space saved: {(1 - 1/overall_ratio)*100:.1f}%")
        if total_messages > 0:
            avg_compress = (total_compress_time / total_messages) * 1000
            avg_decompress = (total_decompress_time / total_messages) * 1000
            print(f"\nTiming Statistics:")
            print(f"Total compression time: {total_compress_time:.2f}s")
            print(f"Total decompression time: {total_decompress_time:.2f}s")
            print(f"Average compression time: {avg_compress:.1f}ms per message")
            print(f"Average decompression time: {avg_decompress:.1f}ms per message")
            print(f"Total processing time: {total_compress_time + total_decompress_time:.2f}s")
        print("=" * 80)
 
if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Point cloud compression/decompression tool')
    parser.add_argument('--mode', type=str, choices=['compress', 'decompress', 'visualize'], required=True,
                        help='Mode: compress, decompress, or visualize (process all msgs from bag)')
    parser.add_argument('--compression', type=str, choices=['octree', 'dpcc'], default='octree',
                        help='Compression method: octree (G-PCC-style) or dpcc (neural network)')
    parser.add_argument('--dpcc-model', type=str, 
                        help='Path to D-PCC model checkpoint (required for dpcc compression)')
    parser.add_argument('--dpcc-downsample', type=float, default=1.0,
                        help='D-PCC downsample rate per layer (0.33-1.0, higher=less lossy, default: 1.0=no downsample)')
    parser.add_argument('--dpcc-upsample', type=int, default=32,
                        help='D-PCC max upsample points (8-64, higher=less lossy, default: 32)')
    parser.add_argument('--rosbag', type=str, 
                        default='~/file_exchange_port/School-Playground/subsea_robotics_intro/2025-12-03_14-34-56_riser_run1',
                        help='Path to rosbag directory (for compress mode)')
    parser.add_argument('--input', type=str, help='Input file (for decompress mode)')
    parser.add_argument('--output', type=str, help='Output file')
    parser.add_argument('--min-range', type=float, default=0.5,
                        help='Minimum range for filtering (meters, default: 0.5)')
    parser.add_argument('--max-range', type=float, default=10.0,
                        help='Maximum range for filtering (meters, default: 10.0)')
    parser.add_argument('--leaf-size', type=float, default=0.10,
                        help='Voxel leaf size for downsampling (meters, default: 0.10)')
    
    args = parser.parse_args()
    
    if args.mode == 'compress':
        # Initialize compressor if using D-PCC
        dpcc_compressor = None
        if args.compression == 'dpcc':
            if not DPCC_AVAILABLE:
                print("Error: D-PCC not available. Please check installation.")
                exit(1)
            if not args.dpcc_model:
                print("Error: --dpcc-model required for D-PCC compression")
                exit(1)
            try:
                dpcc_compressor = DPCCCompressor(
                    args.dpcc_model,
                    downsample_rate=args.dpcc_downsample,
                    max_upsample=args.dpcc_upsample
                )
                if not dpcc_compressor.model_loaded:
                    print("Error: Failed to load D-PCC model")
                    exit(1)
            except Exception as e:
                print(f"Error initializing D-PCC: {e}")
                exit(1)
        
        # Load point cloud from rosbag
        rosbag_path = Path(args.rosbag).expanduser()
        print(f"Loading first lidar message from {rosbag_path}...")
        point_cloud = load_first_lidar_msg(str(rosbag_path))
        print(f"Loaded point cloud with {point_cloud.shape[0]} points")
        
        # Apply range filter
        print(f"Applying range filter ({args.min_range}m - {args.max_range}m)...")
        point_cloud = filter_by_range(point_cloud, args.min_range, args.max_range)
        print(f"After range filter: {point_cloud.shape[0]} points")
        
        # Apply voxel downsampling
        print(f"Applying voxel downsampling (leaf size: {args.leaf_size}m)...")
        point_cloud = voxel_downsample(point_cloud, args.leaf_size)
        print(f"After downsampling: {point_cloud.shape[0]} points")
        
        # Compress using selected method
        print(f"Compressing with {args.compression.upper()}...")
        if args.compression == 'dpcc':
            compressed = dpcc_compressor.compress(point_cloud)
        else:
            compressed = compress_point_cloud(point_cloud)
        
        # Save compressed data
        extension = '.dpcc' if args.compression == 'dpcc' else '.zst'
        output_file = args.output or f'compressed_pointcloud{extension}'
        with open(output_file, 'wb') as f:
            f.write(compressed)
        
        orig_size = point_cloud.nbytes
        comp_size = len(compressed)
        ratio = orig_size / comp_size
        print(f"Original size: {orig_size:,} bytes")
        print(f"Compressed size: {comp_size:,} bytes")
        print(f"Compression ratio: {ratio:.2f}x")
        print(f"Saved to {output_file}")
        
    elif args.mode == 'decompress':
        if not args.input:
            print("Error: --input required for decompress mode")
            exit(1)
        
        # Detect compression method from file extension if not specified
        input_path = Path(args.input)
        if args.compression == 'octree':
            # Auto-detect from extension
            if input_path.suffix == '.dpcc':
                args.compression = 'dpcc'
        
        # Initialize decompressor if using D-PCC
        dpcc_compressor = None
        if args.compression == 'dpcc':
            if not DPCC_AVAILABLE:
                print("Error: D-PCC not available. Please check installation.")
                exit(1)
            if not args.dpcc_model:
                print("Error: --dpcc-model required for D-PCC decompression")
                exit(1)
            try:
                dpcc_compressor = DPCCCompressor(
                    args.dpcc_model,
                    downsample_rate=args.dpcc_downsample,
                    max_upsample=args.dpcc_upsample
                )
                if not dpcc_compressor.model_loaded:
                    print("Error: Failed to load D-PCC model")
                    exit(1)
            except Exception as e:
                print(f"Error initializing D-PCC: {e}")
                exit(1)
        
        # Load compressed data
        print(f"Loading compressed data from {args.input}...")
        with open(args.input, 'rb') as f:
            compressed = f.read()
        
        # Decompress using selected method
        print(f"Decompressing with {args.compression.upper()}...")
        if args.compression == 'dpcc':
            point_cloud = dpcc_compressor.decompress(compressed)
        else:
            point_cloud = decompress_point_cloud(compressed)
        print(f"Decompressed point cloud with {point_cloud.shape[0]} points")
        
        # Save decompressed data
        if args.output:
            np.save(args.output, point_cloud)
            print(f"Saved to {args.output}")
        else:
            print("First 10 points:")
            print(point_cloud[:10])
    
    elif args.mode == 'visualize':
        # Process all messages from bag and visualize
        rosbag_path = Path(args.rosbag).expanduser()
        process_and_visualize_bag(
            str(rosbag_path),
            args.min_range,
            args.max_range,
            args.leaf_size,
            args.compression,
            args.dpcc_model,
            args.dpcc_downsample,
            args.dpcc_upsample
        )