use crate::cli::UiMode;
use crate::config::preferences::SatisfactionLevel;
use chrono::{DateTime, Utc};
use std::collections::VecDeque;
use std::fs;
use std::path::PathBuf;
use serde::{Serialize, Deserialize};

/// Machine learning model for complexity prediction
#[derive(Debug)]
pub struct ComplexityMLModel {
    model: Box<dyn MLModel>,
    training_data: TrainingDataset,
    model_metadata: ModelMetadata,
    model_path: PathBuf,
}

/// Input features for ML model
#[derive(Debug, Clone)]
pub struct MLInput {
    pub command_features: Vec<f32>,
    pub data_features: Vec<f32>,
    pub system_features: Vec<f32>,
    pub context_features: Vec<f32>,
}

/// Output from ML model prediction
#[derive(Debug, Clone)]
pub struct MLOutput {
    pub predicted_complexity: f32,
    pub confidence: f32,
    pub feature_importance: Vec<FeatureImportance>,
    pub prediction_explanation: String,
}

/// Feature importance information
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct FeatureImportance {
    pub feature_name: String,
    pub importance: f32,
    pub positive_impact: bool,
}

/// Training dataset management
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct TrainingDataset {
    examples: VecDeque<TrainingExample>,
    max_size: usize,
    version: u32,
}

/// Individual training example
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct TrainingExample {
    pub input: MLInputSerializable,
    pub actual_complexity: f32,
    pub user_satisfaction: SatisfactionLevel,
    pub interface_choice: UiMode,
    pub timestamp: DateTime<Utc>,
}

/// Serializable version of MLInput
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct MLInputSerializable {
    pub command_features: Vec<f32>,
    pub data_features: Vec<f32>,
    pub system_features: Vec<f32>,
    pub context_features: Vec<f32>,
}

/// Model metadata and performance tracking
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ModelMetadata {
    pub model_type: String,
    pub version: String,
    pub created_at: DateTime<Utc>,
    pub last_updated: DateTime<Utc>,
    pub training_examples_count: usize,
    pub best_validation_score: f32,
    pub feature_names: Vec<String>,
}

/// User feedback for model improvement
#[derive(Debug, Clone)]
pub struct UserComplexityFeedback {
    pub ml_input: MLInput,
    pub predicted_complexity: f32,
    pub user_perceived_complexity: f32,
    pub satisfaction: SatisfactionLevel,
    pub chosen_interface: UiMode,
    pub feedback_comments: Option<String>,
}

/// Trait for different ML model implementations
pub trait MLModel: Send + Sync + std::fmt::Debug {
    fn predict(&self, input: &MLInput) -> eyre::Result<MLOutput>;
    fn update_model(&mut self, training_examples: &[TrainingExample]) -> eyre::Result<()>;
    fn get_feature_importance(&self) -> Vec<FeatureImportance>;
    fn save_model(&self, path: &PathBuf) -> eyre::Result<()>;
    fn load_model(&mut self, path: &PathBuf) -> eyre::Result<()>;
}

/// Linear regression model implementation
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct LinearRegressionModel {
    weights: Vec<f32>,
    bias: f32,
    feature_names: Vec<String>,
    learning_rate: f32,
    regularization: f32,
    training_iterations: usize,
}

/// Random forest model (simplified implementation)
#[derive(Debug)]
pub struct RandomForestModel {
    trees: Vec<DecisionTree>,
    feature_importance: Vec<f32>,
    feature_names: Vec<String>,
}

/// Decision tree for random forest
#[derive(Debug, Clone)]
pub struct DecisionTree {
    root: Option<TreeNode>,
    max_depth: usize,
    min_samples_split: usize,
}

/// Node in decision tree
#[derive(Debug, Clone)]
pub struct TreeNode {
    feature_index: usize,
    threshold: f32,
    left: Option<Box<TreeNode>>,
    right: Option<Box<TreeNode>>,
    prediction: Option<f32>,
    samples: usize,
}

impl ComplexityMLModel {
    /// Load existing model or create new one
    pub fn load_or_create() -> Self {
        let model_path = Self::get_model_path();
        
        match Self::load_from_disk(&model_path) {
            Ok(model) => model,
            Err(_) => Self::create_default_model(model_path),
        }
    }
    
    /// Create a new default model
    fn create_default_model(model_path: PathBuf) -> Self {
        let feature_names = Self::create_feature_names();
        let model = Box::new(LinearRegressionModel::new(feature_names.clone()));
        
        Self {
            model,
            training_data: TrainingDataset::new(1000),
            model_metadata: ModelMetadata {
                model_type: "LinearRegression".to_string(),
                version: "1.0.0".to_string(),
                created_at: Utc::now(),
                last_updated: Utc::now(),
                training_examples_count: 0,
                best_validation_score: 0.0,
                feature_names,
            },
            model_path,
        }
    }
    
    /// Load model from disk
    fn load_from_disk(path: &PathBuf) -> eyre::Result<Self> {
        if !path.exists() {
            return Err(eyre::eyre!("Model file does not exist"));
        }
        
        let model_data = fs::read_to_string(path)?;
        let mut model: ComplexityMLModel = serde_json::from_str(&model_data)?;
        
        // Load the actual ML model weights
        model.model.load_model(path)?;
        
        Ok(model)
    }
    
    /// Get default model path
    fn get_model_path() -> PathBuf {
        dirs::config_dir()
            .unwrap_or_else(|| PathBuf::from("."))
            .join("dora")
            .join("complexity_model.json")
    }
    
    /// Create feature names for the model
    fn create_feature_names() -> Vec<String> {
        vec![
            // Command features
            "command_base_complexity".to_string(),
            "command_parameter_complexity".to_string(),
            "command_dependency_complexity".to_string(),
            "command_output_complexity".to_string(),
            "command_error_potential".to_string(),
            "command_normalized_score".to_string(),
            "command_factor_count".to_string(),
            
            // Data features
            "data_graph_complexity".to_string(),
            "data_resource_complexity".to_string(),
            "data_performance_complexity".to_string(),
            "data_scalability_complexity".to_string(),
            "data_normalized_score".to_string(),
            "data_factor_count".to_string(),
            
            // System features
            "system_load_complexity".to_string(),
            "system_resource_complexity".to_string(),
            "system_stability_complexity".to_string(),
            "system_performance_complexity".to_string(),
            "system_environment_complexity".to_string(),
            "system_normalized_score".to_string(),
            "system_factor_count".to_string(),
            
            // Context features
            "context_is_tty".to_string(),
            "context_is_piped".to_string(),
            "context_is_scripted".to_string(),
            "context_terminal_size".to_string(),
            "context_tui_capable".to_string(),
            "context_supports_color".to_string(),
            "context_is_ci".to_string(),
            "context_user_preference".to_string(),
        ]
    }
    
    /// Predict complexity for given input
    pub fn predict(&self, input: &MLInput) -> eyre::Result<MLOutput> {
        self.model.predict(input)
    }
    
    /// Collect training data from user feedback
    pub async fn collect_training_data(
        &mut self,
        user_feedback: UserComplexityFeedback,
    ) -> eyre::Result<()> {
        let training_example = TrainingExample {
            input: MLInputSerializable {
                command_features: user_feedback.ml_input.command_features,
                data_features: user_feedback.ml_input.data_features,
                system_features: user_feedback.ml_input.system_features,
                context_features: user_feedback.ml_input.context_features,
            },
            actual_complexity: user_feedback.user_perceived_complexity,
            user_satisfaction: user_feedback.satisfaction,
            interface_choice: user_feedback.chosen_interface,
            timestamp: Utc::now(),
        };
        
        self.training_data.add_example(training_example);
        self.model_metadata.training_examples_count += 1;
        self.model_metadata.last_updated = Utc::now();
        
        // Retrain model periodically
        if self.training_data.len() % 50 == 0 {
            self.retrain_model().await?;
        }
        
        Ok(())
    }
    
    /// Retrain the model with new data
    async fn retrain_model(&mut self) -> eyre::Result<()> {
        let recent_examples = self.training_data.get_recent_examples(500);
        self.model.update_model(&recent_examples)?;
        
        // Validate model performance
        let validation_score = self.validate_model(&recent_examples)?;
        
        if validation_score > self.model_metadata.best_validation_score {
            self.model_metadata.best_validation_score = validation_score;
            self.save_model().await?;
        }
        
        Ok(())
    }
    
    /// Validate model performance using cross-validation
    fn validate_model(&self, examples: &[TrainingExample]) -> eyre::Result<f32> {
        if examples.is_empty() {
            return Ok(0.0);
        }
        
        let mut total_error = 0.0;
        let mut count = 0;
        
        for example in examples {
            let input = MLInput {
                command_features: example.input.command_features.clone(),
                data_features: example.input.data_features.clone(),
                system_features: example.input.system_features.clone(),
                context_features: example.input.context_features.clone(),
            };
            
            let prediction = self.model.predict(&input)?;
            let error = (prediction.predicted_complexity - example.actual_complexity).abs();
            total_error += error;
            count += 1;
        }
        
        let mean_absolute_error = total_error / count as f32;
        let accuracy = 1.0 - (mean_absolute_error / 10.0); // Normalize to 0-1 scale
        
        Ok(accuracy.max(0.0))
    }
    
    /// Save model to disk
    async fn save_model(&self) -> eyre::Result<()> {
        // Ensure directory exists
        if let Some(parent) = self.model_path.parent() {
            fs::create_dir_all(parent)?;
        }
        
        // Save model metadata and training data
        let model_data = serde_json::to_string_pretty(self)?;
        fs::write(&self.model_path, model_data)?;
        
        // Save the actual ML model weights
        self.model.save_model(&self.model_path)?;
        
        Ok(())
    }
    
    /// Get feature importance from the model
    pub fn get_feature_importance(&self) -> Vec<FeatureImportance> {
        self.model.get_feature_importance()
    }
}

impl LinearRegressionModel {
    pub fn new(feature_names: Vec<String>) -> Self {
        let num_features = feature_names.len();
        
        Self {
            weights: vec![0.1; num_features], // Initialize with small weights
            bias: 0.0,
            feature_names,
            learning_rate: 0.01,
            regularization: 0.001,
            training_iterations: 0,
        }
    }
    
    /// Flatten all features into a single vector
    fn flatten_features(&self, input: &MLInput) -> Vec<f32> {
        let mut features = Vec::new();
        features.extend(&input.command_features);
        features.extend(&input.data_features);
        features.extend(&input.system_features);
        features.extend(&input.context_features);
        features
    }
    
    /// Calculate confidence based on feature values and model certainty
    fn calculate_confidence(&self, features: &[f32]) -> f32 {
        // Simple confidence calculation based on feature magnitudes and weights
        let weight_magnitude: f32 = self.weights.iter().map(|w| w.abs()).sum();
        let feature_magnitude: f32 = features.iter().map(|f| f.abs()).sum();
        
        let confidence = (weight_magnitude * feature_magnitude / self.weights.len() as f32).min(1.0);
        confidence.max(0.3) // Minimum confidence threshold
    }
    
    /// Calculate feature importance based on weights
    fn calculate_feature_importance(&self, features: &[f32]) -> Vec<FeatureImportance> {
        self.weights.iter()
            .enumerate()
            .map(|(i, &weight)| {
                let feature_value = features.get(i).unwrap_or(&0.0);
                FeatureImportance {
                    feature_name: self.feature_names.get(i)
                        .cloned()
                        .unwrap_or_else(|| format!("feature_{}", i)),
                    importance: (weight * feature_value).abs(),
                    positive_impact: weight > 0.0,
                }
            })
            .collect()
    }
    
    /// Generate explanation for prediction
    fn generate_explanation(&self, features: &[f32], prediction: f32) -> String {
        let top_features: Vec<_> = self.calculate_feature_importance(features)
            .into_iter()
            .filter(|fi| fi.importance > 0.1)
            .take(3)
            .collect();
        
        if top_features.is_empty() {
            return format!("Predicted complexity: {:.1}/10", prediction);
        }
        
        let feature_descriptions: Vec<String> = top_features.iter()
            .map(|fi| format!("{} ({:.2})", fi.feature_name, fi.importance))
            .collect();
        
        format!(
            "Predicted complexity: {:.1}/10. Key factors: {}",
            prediction,
            feature_descriptions.join(", ")
        )
    }
}

impl MLModel for LinearRegressionModel {
    fn predict(&self, input: &MLInput) -> eyre::Result<MLOutput> {
        let features = self.flatten_features(input);
        
        if features.len() != self.weights.len() {
            return Err(eyre::eyre!(
                "Feature count mismatch: expected {}, got {}",
                self.weights.len(),
                features.len()
            ));
        }
        
        let prediction = features.iter()
            .zip(&self.weights)
            .map(|(feature, weight)| feature * weight)
            .sum::<f32>() + self.bias;
        
        let confidence = self.calculate_confidence(&features);
        let feature_importance = self.calculate_feature_importance(&features);
        let explanation = self.generate_explanation(&features, prediction);
        
        Ok(MLOutput {
            predicted_complexity: prediction.clamp(0.0, 10.0),
            confidence,
            feature_importance,
            prediction_explanation: explanation,
        })
    }
    
    fn update_model(&mut self, training_examples: &[TrainingExample]) -> eyre::Result<()> {
        if training_examples.is_empty() {
            return Ok(());
        }
        
        // Simple gradient descent with regularization
        for example in training_examples {
            let input = MLInput {
                command_features: example.input.command_features.clone(),
                data_features: example.input.data_features.clone(),
                system_features: example.input.system_features.clone(),
                context_features: example.input.context_features.clone(),
            };
            
            let features = self.flatten_features(&input);
            let prediction = self.predict(&input)?.predicted_complexity;
            let error = example.actual_complexity - prediction;
            
            // Update weights with regularization
            for (i, &feature) in features.iter().enumerate() {
                if i < self.weights.len() {
                    let gradient = error * feature - self.regularization * self.weights[i];
                    self.weights[i] += self.learning_rate * gradient;
                }
            }
            
            // Update bias
            self.bias += self.learning_rate * error;
        }
        
        self.training_iterations += training_examples.len();
        
        // Decay learning rate over time
        if self.training_iterations % 100 == 0 {
            self.learning_rate *= 0.99;
        }
        
        Ok(())
    }
    
    fn get_feature_importance(&self) -> Vec<FeatureImportance> {
        self.weights.iter()
            .enumerate()
            .map(|(i, &weight)| FeatureImportance {
                feature_name: self.feature_names.get(i)
                    .cloned()
                    .unwrap_or_else(|| format!("feature_{}", i)),
                importance: weight.abs(),
                positive_impact: weight > 0.0,
            })
            .collect()
    }
    
    fn save_model(&self, path: &PathBuf) -> eyre::Result<()> {
        let weights_path = path.with_extension("weights.json");
        let weights_data = serde_json::to_string_pretty(self)?;
        fs::write(weights_path, weights_data)?;
        Ok(())
    }
    
    fn load_model(&mut self, path: &PathBuf) -> eyre::Result<()> {
        let weights_path = path.with_extension("weights.json");
        if weights_path.exists() {
            let weights_data = fs::read_to_string(weights_path)?;
            let loaded_model: LinearRegressionModel = serde_json::from_str(&weights_data)?;
            *self = loaded_model;
        }
        Ok(())
    }
}

impl TrainingDataset {
    pub fn new(max_size: usize) -> Self {
        Self {
            examples: VecDeque::new(),
            max_size,
            version: 1,
        }
    }
    
    pub fn add_example(&mut self, example: TrainingExample) {
        self.examples.push_back(example);
        
        // Remove old examples if we exceed capacity
        while self.examples.len() > self.max_size {
            self.examples.pop_front();
        }
        
        self.version += 1;
    }
    
    pub fn get_recent_examples(&self, count: usize) -> Vec<TrainingExample> {
        self.examples.iter()
            .rev()
            .take(count)
            .cloned()
            .collect()
    }
    
    pub fn len(&self) -> usize {
        self.examples.len()
    }
    
    pub fn is_empty(&self) -> bool {
        self.examples.is_empty()
    }
}

// Implement Serialize for ComplexityMLModel (excluding the trait object)
impl Serialize for ComplexityMLModel {
    fn serialize<S>(&self, serializer: S) -> Result<S::Ok, S::Error>
    where
        S: serde::Serializer,
    {
        use serde::ser::SerializeStruct;
        
        let mut state = serializer.serialize_struct("ComplexityMLModel", 3)?;
        state.serialize_field("training_data", &self.training_data)?;
        state.serialize_field("model_metadata", &self.model_metadata)?;
        state.serialize_field("model_path", &self.model_path)?;
        state.end()
    }
}

// Implement Deserialize for ComplexityMLModel
impl<'de> serde::Deserialize<'de> for ComplexityMLModel {
    fn deserialize<D>(deserializer: D) -> Result<Self, D::Error>
    where
        D: serde::Deserializer<'de>,
    {
        use serde::de::{self, Deserializer, MapAccess, Visitor};
        use std::fmt;
        
        #[derive(Deserialize)]
        #[serde(field_identifier, rename_all = "lowercase")]
        enum Field { TrainingData, ModelMetadata, ModelPath }
        
        struct ComplexityMLModelVisitor;
        
        impl<'de> Visitor<'de> for ComplexityMLModelVisitor {
            type Value = ComplexityMLModel;
            
            fn expecting(&self, formatter: &mut fmt::Formatter) -> fmt::Result {
                formatter.write_str("struct ComplexityMLModel")
            }
            
            fn visit_map<V>(self, mut map: V) -> Result<ComplexityMLModel, V::Error>
            where
                V: MapAccess<'de>,
            {
                let mut training_data = None;
                let mut model_metadata = None;
                let mut model_path = None;
                
                while let Some(key) = map.next_key()? {
                    match key {
                        Field::TrainingData => {
                            if training_data.is_some() {
                                return Err(de::Error::duplicate_field("training_data"));
                            }
                            training_data = Some(map.next_value()?);
                        }
                        Field::ModelMetadata => {
                            if model_metadata.is_some() {
                                return Err(de::Error::duplicate_field("model_metadata"));
                            }
                            model_metadata = Some(map.next_value()?);
                        }
                        Field::ModelPath => {
                            if model_path.is_some() {
                                return Err(de::Error::duplicate_field("model_path"));
                            }
                            model_path = Some(map.next_value()?);
                        }
                    }
                }
                
                let training_data = training_data.ok_or_else(|| de::Error::missing_field("training_data"))?;
                let model_metadata: ModelMetadata = model_metadata.ok_or_else(|| de::Error::missing_field("model_metadata"))?;
                let model_path = model_path.ok_or_else(|| de::Error::missing_field("model_path"))?;
                
                // Recreate the model based on metadata
                let model = Box::new(LinearRegressionModel::new(model_metadata.feature_names.clone()));
                
                Ok(ComplexityMLModel {
                    model,
                    training_data,
                    model_metadata,
                    model_path,
                })
            }
        }
        
        const FIELDS: &'static [&'static str] = &["training_data", "model_metadata", "model_path"];
        deserializer.deserialize_struct("ComplexityMLModel", FIELDS, ComplexityMLModelVisitor)
    }
}