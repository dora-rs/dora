#!/usr/bin/env python3
"""
Test audio conversion from int16 bytes to float32
"""

import numpy as np
import sys

def test_conversion():
    print("Testing Int16 to Float32 conversion")
    print("=" * 50)
    
    # Test data: Create known int16 values
    test_values_int16 = np.array([
        0,        # Zero
        100,      # Small positive
        -100,     # Small negative
        16383,    # Half max positive (~0.5 after conversion)
        -16384,   # Half max negative (~-0.5 after conversion)
        32767,    # Max positive (should be ~1.0)
        -32768,   # Max negative (should be -1.0)
    ], dtype=np.int16)
    
    print("Original int16 values:")
    print(test_values_int16)
    
    # Convert to bytes (as the library provides)
    bytes_data = test_values_int16.tobytes()
    print(f"\nBytes representation ({len(bytes_data)} bytes):")
    print(' '.join(f'{b:02x}' for b in bytes_data[:20]))  # Show first 20 bytes
    
    # Method 1: Current conversion in AEC node
    print("\n--- Method 1: Current AEC conversion ---")
    audio_array = np.frombuffer(bytes_data, dtype=np.int16)
    audio_float32 = audio_array.astype(np.float32) / 32768.0
    print("Float32 values:")
    print(audio_float32)
    
    # Check for potential issues
    print("\nPotential issues:")
    
    # Issue 1: Asymmetry
    max_positive = 32767 / 32768.0  # 0.999969...
    max_negative = -32768 / 32768.0  # -1.0
    print(f"1. Asymmetry: max positive = {max_positive:.6f}, max negative = {max_negative:.6f}")
    
    # Issue 2: Endianness
    print("\n2. Endianness check:")
    print(f"   System byte order: {sys.byteorder}")
    
    # Test with wrong endianness
    audio_array_swapped = np.frombuffer(bytes_data, dtype='>i2')  # Big-endian
    audio_float32_swapped = audio_array_swapped.astype(np.float32) / 32768.0
    print(f"   If endianness is wrong, first value would be: {audio_float32_swapped[0]:.6f}")
    
    # Method 2: Alternative conversion (symmetric)
    print("\n--- Method 2: Symmetric conversion ---")
    audio_float32_alt = np.clip(audio_array.astype(np.float32) / 32767.0, -1.0, 1.0)
    print("Float32 values (symmetric):")
    print(audio_float32_alt)
    
    # Method 3: Using numpy's built-in conversion
    print("\n--- Method 3: NumPy int16 to float32 ---")
    # This is what some libraries use
    audio_float32_np = audio_array.astype(np.float32)
    # Then normalize
    max_val = np.abs(audio_float32_np).max()
    if max_val > 0:
        audio_float32_np = audio_float32_np / max_val
    print("Float32 values (normalized):")
    print(audio_float32_np)
    
    # Test with actual audio-like data
    print("\n--- Test with sine wave ---")
    t = np.linspace(0, 1, 100)
    sine_int16 = (np.sin(2 * np.pi * 440 * t) * 32767).astype(np.int16)
    sine_bytes = sine_int16.tobytes()
    
    # Convert using current method
    sine_array = np.frombuffer(sine_bytes, dtype=np.int16)
    sine_float32 = sine_array.astype(np.float32) / 32768.0
    
    print(f"Sine wave stats:")
    print(f"  Int16 range: [{sine_int16.min()}, {sine_int16.max()}]")
    print(f"  Float32 range: [{sine_float32.min():.6f}, {sine_float32.max():.6f}]")
    print(f"  Float32 mean: {sine_float32.mean():.6f}")
    print(f"  Float32 std: {sine_float32.std():.6f}")
    
    # Check if clipping would occur
    if np.any(np.abs(sine_float32) > 1.0):
        print("  WARNING: Values exceed [-1, 1] range!")
    else:
        print("  ✓ All values within [-1, 1] range")

if __name__ == "__main__":
    test_conversion()