# Examples and Tutorials

This directory contains various examples and tutorials to help you get started with Dora.

## Basic Examples
- [Basic Node Creation](basic/README.md)
- [Data Transformation](basic/data_transformation.md)
- [Simple Pipeline](basic/pipeline.md)

## Advanced Examples
- [Image Processing Pipeline](advanced/image_processing.md)
- [Real-time Data Processing](advanced/realtime_processing.md)
- [API Integration](advanced/api_integration.md)

## Performance Examples
- [Batch Processing](advanced/batch_processing.md)
- [Parallel Processing](advanced/parallel_processing.md)
- [Memory Optimization](advanced/memory_optimization.md)

## End-to-End Examples
- [Complete Data Pipeline](advanced/complete_pipeline.md)
- [Real-time Monitoring System](advanced/monitoring_system.md)

## Real-World Use Cases

### Image Processing Pipeline
```python
from dora import Node, DataFlow
import cv2
import numpy as np

class ImageLoader(Node):
    async def on_input(self, data):
        image = cv2.imread(data["path"])
        return {"image": image}

class ImageProcessor(Node):
    async def on_input(self, data):
        image = data["image"]
        # Apply image processing
        processed = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        return {"processed": processed}

class ImageSaver(Node):
    async def on_input(self, data):
        cv2.imwrite(data["output_path"], data["processed"])
        return {"status": "success"}

# Create pipeline
flow = DataFlow()
flow.add_node("loader", ImageLoader)
flow.add_node("processor", ImageProcessor)
flow.add_node("saver", ImageSaver)

# Connect nodes
flow.connect("loader.output", "processor.input")
flow.connect("processor.output", "saver.input")

# Run pipeline
flow.run()
```

### Real-time Data Processing
```python
class SensorReader(Node):
    async def on_input(self, data):
        # Read from sensor
        value = read_sensor()
        return {"value": value}

class DataAnalyzer(Node):
    async def on_input(self, data):
        value = data["value"]
        # Analyze data
        analysis = analyze_data(value)
        return {"analysis": analysis}

class AlertSystem(Node):
    async def on_input(self, data):
        if data["analysis"]["anomaly"]:
            send_alert(data["analysis"])
        return {"status": "processed"}
```

### API Integration
```python
import aiohttp

class APIClient(Node):
    async def on_input(self, data):
        async with aiohttp.ClientSession() as session:
            async with session.get(data["url"]) as response:
                result = await response.json()
                return {"api_response": result}

class ResponseProcessor(Node):
    async def on_input(self, data):
        response = data["api_response"]
        # Process API response
        processed = process_response(response)
        return {"processed": processed}
```

## Performance Optimization Examples

### Batch Processing
```python
class BatchProcessor(Node):
    def __init__(self, config):
        super().__init__(config)
        self.batch_size = config.get("batch_size", 100)
        self.buffer = []

    async def on_input(self, data):
        self.buffer.append(data)
        if len(self.buffer) >= self.batch_size:
            result = process_batch(self.buffer)
            self.buffer = []
            return {"batch_result": result}
        return None
```

### Parallel Processing
```python
import asyncio

class ParallelProcessor(Node):
    async def on_input(self, data):
        tasks = []
        for item in data["items"]:
            tasks.append(process_item(item))
        results = await asyncio.gather(*tasks)
        return {"results": results}
```

### Memory Optimization
```python
class MemoryEfficientNode(Node):
    def __init__(self, config):
        super().__init__(config)
        self.pool = MemoryPool()

    async def on_input(self, data):
        with self.pool.allocate() as buffer:
            result = process_with_buffer(data, buffer)
            return {"result": result}
```

## End-to-End Examples

### Complete Data Pipeline
```python
from dora import Node, DataFlow
import pandas as pd
from sklearn.preprocessing import StandardScaler

class DataLoader(Node):
    async def on_input(self, data):
        df = pd.read_csv(data["file_path"])
        return {"data": df}

class DataPreprocessor(Node):
    async def on_input(self, data):
        df = data["data"]
        scaler = StandardScaler()
        scaled_data = scaler.fit_transform(df)
        return {"scaled_data": scaled_data}

class ModelPredictor(Node):
    async def on_input(self, data):
        predictions = model.predict(data["scaled_data"])
        return {"predictions": predictions}

class ResultAggregator(Node):
    async def on_input(self, data):
        results = aggregate_results(data["predictions"])
        return {"final_results": results}

# Create and run pipeline
flow = DataFlow()
flow.add_node("loader", DataLoader)
flow.add_node("preprocessor", DataPreprocessor)
flow.add_node("predictor", ModelPredictor)
flow.add_node("aggregator", ResultAggregator)

# Connect nodes
flow.connect("loader.output", "preprocessor.input")
flow.connect("preprocessor.output", "predictor.input")
flow.connect("predictor.output", "aggregator.input")

# Run pipeline
flow.run()
```

### Real-time Monitoring System
```python
class MetricsCollector(Node):
    async def on_input(self, data):
        metrics = collect_system_metrics()
        return {"metrics": metrics}

class AnomalyDetector(Node):
    async def on_input(self, data):
        anomalies = detect_anomalies(data["metrics"])
        return {"anomalies": anomalies}

class AlertManager(Node):
    async def on_input(self, data):
        if data["anomalies"]:
            send_alerts(data["anomalies"])
        return {"status": "processed"}

class DashboardUpdater(Node):
    async def on_input(self, data):
        update_dashboard(data["metrics"])
        return {"status": "updated"}
```

## Best Practices

### Error Handling
```python
class RobustNode(Node):
    async def on_input(self, data):
        try:
            result = process_data(data)
            return {"result": result}
        except ValueError as e:
            self.logger.error(f"Invalid input: {e}")
            return None
        except Exception as e:
            self.logger.error(f"Unexpected error: {e}")
            raise
```

### Configuration Management
```python
class ConfigurableNode(Node):
    def __init__(self, config):
        super().__init__(config)
        self.threshold = config.get("threshold", 100)
        self.timeout = config.get("timeout", 30)
        self.retry_count = config.get("retry_count", 3)
```

### Logging and Monitoring
```python
class MonitoredNode(Node):
    async def on_input(self, data):
        self.logger.info("Processing input")
        start_time = time.time()
        
        result = process_data(data)
        
        duration = time.time() - start_time
        self.logger.info(f"Processing completed in {duration:.2f}s")
        return {"result": result}
``` 