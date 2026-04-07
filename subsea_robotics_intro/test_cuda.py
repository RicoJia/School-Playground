#!/usr/bin/env python3
"""Isolated CUDA test to diagnose initialization issues."""

import os
import sys

# Try to initialize CUDA before any other imports
print("Setting CUDA environment...")
os.environ['CUDA_LAUNCH_BLOCKING'] = '1'
os.environ['TORCH_USE_CUDA_DSA'] = '1'

print("Importing torch...")
import torch

print(f"PyTorch version: {torch.__version__}")
print(f"CUDA compiled version: {torch.version.cuda}")
print(f"cuDNN version: {torch.backends.cudnn.version()}")

print("\nAttempting CUDA initialization...")
try:
    # Force CUDA initialization
    device_count = torch.cuda.device_count()
    print(f"✓ Device count: {device_count}")
    
    if device_count > 0:
        print(f"✓ Device 0 name: {torch.cuda.get_device_name(0)}")
        print(f"✓ Device 0 capability: {torch.cuda.get_device_capability(0)}")
        
        # Try creating a simple tensor on GPU
        print("\nTrying to create GPU tensor...")
        x = torch.tensor([1.0, 2.0, 3.0], device='cuda')
        print(f"✓ Created tensor on GPU: {x}")
        
        print("\nTrying basic GPU operation...")
        y = x * 2
        print(f"✓ GPU operation successful: {y}")
        
        print("\n✅ CUDA is working correctly!")
    else:
        print("⚠️ No CUDA devices found")
        
except Exception as e:
    print(f"\n❌ CUDA initialization failed:")
    print(f"Error: {e}")
    import traceback
    traceback.print_exc()
    
    # Try to get more detailed error info
    print("\n--- Detailed diagnostics ---")
    try:
        import torch.cuda
        print(f"CUDA available: {torch.cuda.is_available()}")
    except Exception as e2:
        print(f"torch.cuda.is_available() failed: {e2}")
    
    sys.exit(1)
