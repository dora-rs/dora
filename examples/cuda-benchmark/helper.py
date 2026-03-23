"""Helper utilities for CUDA benchmarking in dora-rs.

Provides functions to record, calculate statistics (mean, quantiles),
and log benchmark results to CSV files for performance analysis.
"""

import csv
import os
import platform
from datetime import datetime

import numpy as np
from dora import __version__

LATENCY = True

DATE = str(datetime.now())
LANGUAGE = f"Python {platform.python_version()}"
PLATFORM = platform.platform()
DORA_VERSION = __version__
LOG_HEADER = [
    "Date",
    "Language",
    "Dora Version",
    "Platform",
    "Name",
    "Size (bit)",
    "Latency (ns)",
    "Q1 lat (ns)",
    "Med lat (ns)",
    "Q3 (ns)",
]


def record_results(name, current_size, latencies):
    """Record benchmark results and save them to a CSV file.

    Calculates the mean, Q1, median, and Q3 latencies from the provided
    data and appends the results to a CSV log.

    Args:
        name (str): The name of the benchmark test.
        current_size (int): The size of the data packets being tested.
        latencies (List[float]): A list of measured latency values.
    """
    avg_latency = np.array(latencies).mean()

    # Calculate Q1 (25th percentile), median (50th percentile), and Q3 (75th percentile)
    q1 = np.percentile(latencies, 25)
    median = np.percentile(latencies, 50)  # Alternatively, np.median(data)
    q3 = np.percentile(latencies, 75)

    csv_file = os.getenv("CSV_TIME_FILE", "benchmark_data.csv")
    append = os.path.isfile(csv_file)
    log_row = [
        DATE,
        LANGUAGE,
        DORA_VERSION,
        PLATFORM,
        name,
        current_size,
        int(avg_latency),
        int(q1),
        int(median),
        int(q3),
    ]
    if append:
        with open(csv_file, "a", encoding="utf-8") as f:
            w = csv.writer(f, lineterminator="\n")
            w.writerow(log_row)
    else:
        with open(csv_file, "w+", encoding="utf-8") as f:
            w = csv.writer(f, lineterminator="\n")
            w.writerow(LOG_HEADER)
            w.writerow(log_row)
