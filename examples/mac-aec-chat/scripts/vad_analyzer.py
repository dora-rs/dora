#!/usr/bin/env python3
"""
Voice Activity Detection analyzer
Tracks VAD patterns and provides statistics
"""

import time
import pyarrow as pa
from dora import Node
from collections import deque
import json


class VADAnalyzer:
    def __init__(self):
        self.node = Node()
        
        # VAD tracking
        self.vad_history = deque(maxlen=1000)  # Keep last 1000 VAD states
        self.current_segment = None
        self.segments = []
        
        # Statistics
        self.start_time = time.time()
        self.last_report = time.time()
        self.report_interval = 5.0  # Report every 5 seconds
        
        # VAD state tracking
        self.last_vad_state = False
        self.segment_start = None
        
    def process_vad(self, vad_active):
        """Process VAD state change"""
        current_time = time.time()
        
        # Record VAD state with timestamp
        self.vad_history.append((current_time, vad_active))
        
        # Detect state changes
        if vad_active != self.last_vad_state:
            if vad_active:
                # Speech started
                self.segment_start = current_time
                self.current_segment = {
                    'start': current_time,
                    'type': 'speech'
                }
                print(f"[{current_time:.2f}] Speech started")
                
            else:
                # Speech ended
                if self.segment_start is not None:
                    duration = current_time - self.segment_start
                    segment = {
                        'start': self.segment_start,
                        'end': current_time,
                        'duration': duration,
                        'type': 'speech'
                    }
                    self.segments.append(segment)
                    print(f"[{current_time:.2f}] Speech ended (duration: {duration:.2f}s)")
                    
                    # Send segment info
                    self.node.send_output("vad_segments", pa.array([json.dumps(segment)]))
                
                self.current_segment = None
                self.segment_start = None
        
        self.last_vad_state = vad_active
    
    def calculate_stats(self):
        """Calculate VAD statistics"""
        if not self.vad_history:
            return {}
        
        current_time = time.time()
        total_time = current_time - self.start_time
        
        # Calculate activity rates
        active_count = sum(1 for _, active in self.vad_history if active)
        activity_rate = active_count / len(self.vad_history) if self.vad_history else 0
        
        # Segment statistics
        speech_segments = [s for s in self.segments if s['type'] == 'speech']
        total_speech_time = sum(s['duration'] for s in speech_segments)
        
        avg_segment_duration = (
            sum(s['duration'] for s in speech_segments) / len(speech_segments)
            if speech_segments else 0
        )
        
        # Recent activity (last 30 seconds)
        recent_cutoff = current_time - 30.0
        recent_vad = [active for ts, active in self.vad_history if ts >= recent_cutoff]
        recent_activity = sum(recent_vad) / len(recent_vad) if recent_vad else 0
        
        stats = {
            'session_duration': total_time,
            'total_segments': len(speech_segments),
            'total_speech_time': total_speech_time,
            'speech_ratio': total_speech_time / total_time if total_time > 0 else 0,
            'avg_segment_duration': avg_segment_duration,
            'activity_rate': activity_rate,
            'recent_activity': recent_activity,
            'current_state': 'speaking' if self.last_vad_state else 'silent',
            'buffer_size': len(self.vad_history)
        }
        
        return stats
    
    def format_report(self, stats):
        """Format statistics report"""
        return f"""
╔════════════════════════════════════════════════╗
║               VAD ANALYZER REPORT              ║
╠════════════════════════════════════════════════╣
║ Session time:    {stats['session_duration']:7.1f}s              ║
║ Current state:   {stats['current_state']:12s}          ║
║ Speech segments: {stats['total_segments']:6d}                    ║
║ Total speech:    {stats['total_speech_time']:7.1f}s              ║
║ Speech ratio:    {stats['speech_ratio']*100:6.1f}%              ║
║ Avg segment:     {stats['avg_segment_duration']:7.1f}s              ║
║ Activity rate:   {stats['activity_rate']*100:6.1f}%              ║
║ Recent activity: {stats['recent_activity']*100:6.1f}%              ║
╚════════════════════════════════════════════════╝"""
    
    def run(self):
        """Main processing loop"""
        print("Starting VAD analyzer...")
        print("Monitoring voice activity patterns...")
        
        try:
            while True:
                event = self.node.next(timeout=0.1)
                
                if event and event["type"] == "INPUT":
                    if event["id"] == "vad":
                        vad_active = event["value"][0].as_py()
                        self.process_vad(vad_active)
                
                # Send periodic reports
                current_time = time.time()
                if current_time - self.last_report >= self.report_interval:
                    stats = self.calculate_stats()
                    
                    # Print report
                    print("\033[2J\033[H")  # Clear screen
                    print(self.format_report(stats))
                    
                    # Send stats
                    self.node.send_output("vad_stats", pa.array([json.dumps(stats)]))
                    
                    self.last_report = current_time
                    
        except KeyboardInterrupt:
            print("\nStopping VAD analyzer...")
            
            # Final report
            stats = self.calculate_stats()
            print("\nFinal VAD Analysis:")
            print(self.format_report(stats))
            
        except Exception as e:
            print(f"VAD analyzer error: {e}")


if __name__ == "__main__":
    analyzer = VADAnalyzer()
    analyzer.run()