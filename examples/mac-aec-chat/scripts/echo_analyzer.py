#!/usr/bin/env python3
"""
Echo cancellation effectiveness analyzer
Compares reference audio with AEC output to measure cancellation
"""

import numpy as np
import pyarrow as pa
from dora import Node
import time
from collections import deque
import json


class EchoAnalyzer:
    def __init__(self):
        self.node = Node()
        
        # Configuration
        self.sample_rate = 16000
        self.window_size_ms = int(os.getenv("WINDOW_SIZE_MS", "100"))
        self.report_interval = float(os.getenv("REPORT_INTERVAL", "1.0"))
        
        self.window_samples = int(self.window_size_ms * self.sample_rate / 1000)
        
        # Audio buffers
        self.reference_buffer = deque(maxlen=self.window_samples * 2)
        self.captured_buffer = deque(maxlen=self.window_samples * 2)
        
        # Analysis state
        self.is_playing = False
        self.last_report = time.time()
        
        # Metrics tracking
        self.metrics_history = deque(maxlen=100)
        
        print(f"Echo analyzer initialized:")
        print(f"  Window size: {self.window_size_ms}ms ({self.window_samples} samples)")
        print(f"  Report interval: {self.report_interval}s")
    
    def calculate_cross_correlation(self, ref_audio, captured_audio):
        """Calculate cross-correlation to find echo delay"""
        if len(ref_audio) == 0 or len(captured_audio) == 0:
            return 0, 0
        
        # Normalize audio
        ref_norm = ref_audio / (np.std(ref_audio) + 1e-8)
        cap_norm = captured_audio / (np.std(captured_audio) + 1e-8)
        
        # Cross-correlation
        correlation = np.correlate(cap_norm, ref_norm, mode='full')
        
        # Find peak correlation and delay
        peak_idx = np.argmax(np.abs(correlation))
        delay = peak_idx - len(ref_norm) + 1
        max_correlation = correlation[peak_idx]
        
        return delay, abs(max_correlation)
    
    def calculate_echo_reduction(self, ref_audio, captured_audio):
        """Calculate echo reduction in dB"""
        if len(ref_audio) == 0 or len(captured_audio) == 0:
            return float('inf')  # Perfect cancellation when no reference
        
        # RMS levels
        ref_rms = np.sqrt(np.mean(ref_audio ** 2))
        cap_rms = np.sqrt(np.mean(captured_audio ** 2))
        
        if ref_rms < 1e-6 or cap_rms < 1e-6:
            return float('inf')  # Avoid log of zero
        
        # Reduction in dB (positive = good cancellation)
        reduction_db = 20 * np.log10(ref_rms / cap_rms)
        
        return reduction_db
    
    def analyze_audio_correlation(self):
        """Analyze correlation between reference and captured audio"""
        if len(self.reference_buffer) < self.window_samples or \
           len(self.captured_buffer) < self.window_samples:
            return None
        
        # Get recent audio windows
        ref_audio = np.array(list(self.reference_buffer)[-self.window_samples:])
        cap_audio = np.array(list(self.captured_buffer)[-self.window_samples:])
        
        # Calculate metrics
        delay, correlation = self.calculate_cross_correlation(ref_audio, cap_audio)
        reduction_db = self.calculate_echo_reduction(ref_audio, cap_audio)
        
        # Additional metrics
        ref_rms = np.sqrt(np.mean(ref_audio ** 2))
        cap_rms = np.sqrt(np.mean(cap_audio ** 2))
        ref_peak = np.max(np.abs(ref_audio))
        cap_peak = np.max(np.abs(cap_audio))
        
        metrics = {
            'timestamp': time.time(),
            'is_playing': self.is_playing,
            'delay_samples': int(delay),
            'delay_ms': delay * 1000.0 / self.sample_rate,
            'correlation': float(correlation),
            'reduction_db': float(reduction_db),
            'reference_rms': float(ref_rms),
            'captured_rms': float(cap_rms),
            'reference_peak': float(ref_peak),
            'captured_peak': float(cap_peak),
            'echo_suppression': 'excellent' if reduction_db > 20 else 
                               'good' if reduction_db > 10 else
                               'fair' if reduction_db > 3 else 'poor'
        }
        
        return metrics
    
    def format_metrics_report(self, metrics):
        """Format metrics for display"""
        if not metrics:
            return "No metrics available"
        
        status_color = {
            'excellent': '\033[92m',  # Green
            'good': '\033[93m',       # Yellow  
            'fair': '\033[94m',       # Blue
            'poor': '\033[91m'        # Red
        }
        
        color = status_color.get(metrics['echo_suppression'], '')
        reset = '\033[0m'
        
        return f"""
╔══════════════════════════════════════════════════════╗
║                ECHO CANCELLATION ANALYSIS           ║
╠══════════════════════════════════════════════════════╣
║ Playing:        {'YES' if metrics['is_playing'] else 'NO ':12s}                  ║
║ Suppression:    {color}{metrics['echo_suppression']:12s}{reset}            ║
║ Reduction:      {metrics['reduction_db']:8.1f} dB                  ║
║ Correlation:    {metrics['correlation']:8.3f}                      ║
║ Echo delay:     {metrics['delay_ms']:8.1f} ms                  ║
║                                                      ║
║ Reference RMS:  {metrics['reference_rms']:8.4f}                      ║
║ Captured RMS:   {metrics['captured_rms']:8.4f}                      ║
║ Reference peak: {metrics['reference_peak']:8.4f}                      ║
║ Captured peak:  {metrics['captured_peak']:8.4f}                      ║
╚══════════════════════════════════════════════════════╝"""
    
    def run(self):
        """Main processing loop"""
        print("Starting echo analyzer...")
        print("Analyzing echo cancellation effectiveness...")
        
        try:
            while True:
                event = self.node.next(timeout=0.1)
                
                if event and event["type"] == "INPUT":
                    if event["id"] == "reference":
                        # Store reference audio (what's being played)
                        audio_data = event["value"].to_numpy()
                        self.reference_buffer.extend(audio_data)
                        
                    elif event["id"] == "captured":
                        # Store captured audio (after AEC)
                        audio_data = event["value"].to_numpy()
                        self.captured_buffer.extend(audio_data)
                        
                    elif event["id"] == "is_playing":
                        # Update playback status
                        self.is_playing = event["value"][0].as_py()
                
                # Generate reports
                current_time = time.time()
                if current_time - self.last_report >= self.report_interval:
                    metrics = self.analyze_audio_correlation()
                    
                    if metrics:
                        # Store metrics
                        self.metrics_history.append(metrics)
                        
                        # Display report
                        print("\033[2J\033[H")  # Clear screen
                        print(self.format_metrics_report(metrics))
                        
                        # Send metrics
                        self.node.send_output("echo_metrics", pa.array([json.dumps(metrics)]))
                        self.node.send_output("reduction_db", pa.array([metrics['reduction_db']]))
                    
                    self.last_report = current_time
                    
        except KeyboardInterrupt:
            print("\nStopping echo analyzer...")
            
            # Final summary
            if self.metrics_history:
                avg_reduction = np.mean([m['reduction_db'] for m in self.metrics_history if m['is_playing']])
                print(f"\nSummary: Average echo reduction: {avg_reduction:.1f} dB")
                
        except Exception as e:
            print(f"Echo analyzer error: {e}")


if __name__ == "__main__":
    import os
    analyzer = EchoAnalyzer()
    analyzer.run()