#!/usr/bin/env python3
"""
Test the native AEC library directly
"""

import ctypes
import time

def test_library():
    lib_path = "/Users/yuechen/home/conversation/dora/node-hub/dora-aec/dora_aec/lib/libAudioCapture.dylib"
    
    print(f"Loading library from: {lib_path}")
    
    try:
        # Load the library
        lib = ctypes.CDLL(lib_path)
        print("Library loaded successfully")
        
        # Configure function signatures
        lib.startRecord.argtypes = []
        lib.startRecord.restype = None
        
        lib.stopRecord.argtypes = []
        lib.stopRecord.restype = None
        
        lib.getAudioData.argtypes = [
            ctypes.POINTER(ctypes.c_int),
            ctypes.POINTER(ctypes.c_bool)
        ]
        lib.getAudioData.restype = ctypes.POINTER(ctypes.c_ubyte)
        
        lib.freeAudioData.argtypes = [ctypes.POINTER(ctypes.c_ubyte)]
        lib.freeAudioData.restype = None
        
        print("Function signatures configured")
        
        # Start recording
        print("Starting recording...")
        lib.startRecord()
        
        # Try to get audio data a few times
        for i in range(10):
            time.sleep(0.1)
            
            size = ctypes.c_int(0)
            is_voice = ctypes.c_bool(False)
            
            data_ptr = lib.getAudioData(ctypes.byref(size), ctypes.byref(is_voice))
            
            if data_ptr and size.value > 0:
                print(f"Got audio data: {size.value} bytes, voice active: {is_voice.value}")
                # Free the data
                lib.freeAudioData(data_ptr)
            else:
                print(f"No audio data available (attempt {i+1})")
        
        # Stop recording
        print("Stopping recording...")
        lib.stopRecord()
        
        print("Test completed successfully")
        
    except Exception as e:
        print(f"Error: {e}")
        import traceback
        traceback.print_exc()

if __name__ == "__main__":
    test_library()