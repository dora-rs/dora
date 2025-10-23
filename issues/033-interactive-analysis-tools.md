# Issue #033: Build Interactive Analysis Tools

## 📋 Summary
Implement a comprehensive suite of interactive analysis tools that provide advanced data exploration, pattern discovery, statistical analysis, and predictive modeling capabilities for Dora dataflows and system metrics. These tools enable developers to gain deep insights into system behavior, identify optimization opportunities, and predict potential issues before they occur.

## 🎯 Objectives
- Create interactive data exploration tools with drill-down and correlation analysis
- Implement statistical analysis suite with hypothesis testing and trend detection
- Add machine learning tools for anomaly detection and predictive modeling
- Provide comparative analysis capabilities for A/B testing and performance benchmarking
- Enable automated insight generation with natural language explanations

**Success Metrics:**
- Analysis tool operations complete within 2 seconds for datasets up to 1M points
- Pattern detection accuracy exceeds 90% for known anomaly types
- Predictive models achieve 85%+ accuracy for system behavior forecasting
- User insight discovery rate improves by 3x compared to manual analysis
- Automated explanation quality rated 8/10 or higher by users

## 🛠️ Technical Requirements

### What to Build

#### 1. Interactive Data Explorer
```rust
// src/analysis/data_explorer.rs
#[derive(Debug)]
pub struct InteractiveDataExplorer {
    data_manager: ExplorerDataManager,
    analysis_engine: AnalysisEngine,
    visualization_engine: ExplorerVisualizationEngine,
    correlation_analyzer: CorrelationAnalyzer,
    dimension_reducer: DimensionReducer,
    pattern_detector: PatternDetector,
    filter_engine: FilterEngine,
    drill_down_manager: DrillDownManager,
}

#[derive(Debug, Clone)]
pub struct ExplorationConfig {
    pub data_source: DataSourceConfig,
    pub analysis_type: AnalysisType,
    pub time_range: TimeRange,
    pub dimensions: Vec<Dimension>,
    pub metrics: Vec<Metric>,
    pub filters: Vec<Filter>,
    pub sampling_strategy: SamplingStrategy,
}

#[derive(Debug, Clone)]
pub enum AnalysisType {
    Exploratory {
        auto_discover_patterns: bool,
        correlation_threshold: f64,
        outlier_detection: bool,
    },
    Comparative {
        baseline_period: TimeRange,
        comparison_period: TimeRange,
        statistical_tests: Vec<StatisticalTest>,
    },
    Temporal {
        seasonality_detection: bool,
        trend_analysis: bool,
        changepoint_detection: bool,
    },
    Causal {
        intervention_points: Vec<DateTime<Utc>>,
        control_variables: Vec<String>,
        causal_inference_method: CausalInferenceMethod,
    },
}

impl InteractiveDataExplorer {
    pub fn new() -> Self {
        Self {
            data_manager: ExplorerDataManager::new(),
            analysis_engine: AnalysisEngine::new(),
            visualization_engine: ExplorerVisualizationEngine::new(),
            correlation_analyzer: CorrelationAnalyzer::new(),
            dimension_reducer: DimensionReducer::new(),
            pattern_detector: PatternDetector::new(),
            filter_engine: FilterEngine::new(),
            drill_down_manager: DrillDownManager::new(),
        }
    }
    
    pub async fn start_exploration(&mut self, config: ExplorationConfig) -> Result<ExplorationSession, AnalysisError> {
        // Load and prepare data
        let dataset = self.data_manager.load_dataset(&config.data_source, &config.time_range).await?;
        let prepared_data = self.data_manager.prepare_data(dataset, &config).await?;
        
        // Create exploration session
        let session = ExplorationSession {
            id: ExplorationSessionId::new(),
            config: config.clone(),
            dataset: prepared_data,
            analysis_history: Vec::new(),
            current_view: ExplorationView::Overview,
            bookmarks: Vec::new(),
            insights: Vec::new(),
        };
        
        // Perform initial analysis
        let initial_insights = self.perform_initial_analysis(&session).await?;
        
        // Update session with insights
        let mut updated_session = session;
        updated_session.insights.extend(initial_insights);
        
        Ok(updated_session)
    }
    
    async fn perform_initial_analysis(&mut self, session: &ExplorationSession) -> Result<Vec<Insight>, AnalysisError> {
        let mut insights = Vec::new();
        
        match &session.config.analysis_type {
            AnalysisType::Exploratory { auto_discover_patterns, correlation_threshold, outlier_detection } => {
                // Automatic pattern discovery
                if *auto_discover_patterns {
                    let patterns = self.pattern_detector.discover_patterns(&session.dataset).await?;
                    insights.extend(patterns.into_iter().map(Insight::Pattern));
                }
                
                // Correlation analysis
                let correlations = self.correlation_analyzer.analyze_correlations(&session.dataset, *correlation_threshold).await?;
                insights.extend(correlations.into_iter().map(Insight::Correlation));
                
                // Outlier detection
                if *outlier_detection {
                    let outliers = self.analysis_engine.detect_outliers(&session.dataset).await?;
                    insights.extend(outliers.into_iter().map(Insight::Outlier));
                }
            },
            
            AnalysisType::Comparative { baseline_period, comparison_period, statistical_tests } => {
                let comparison_results = self.perform_comparative_analysis(
                    &session.dataset,
                    baseline_period,
                    comparison_period,
                    statistical_tests
                ).await?;
                insights.extend(comparison_results.into_iter().map(Insight::Comparison));
            },
            
            AnalysisType::Temporal { seasonality_detection, trend_analysis, changepoint_detection } => {
                let temporal_insights = self.perform_temporal_analysis(
                    &session.dataset,
                    *seasonality_detection,
                    *trend_analysis,
                    *changepoint_detection
                ).await?;
                insights.extend(temporal_insights.into_iter().map(Insight::Temporal));
            },
            
            AnalysisType::Causal { intervention_points, control_variables, causal_inference_method } => {
                let causal_results = self.perform_causal_analysis(
                    &session.dataset,
                    intervention_points,
                    control_variables,
                    causal_inference_method
                ).await?;
                insights.extend(causal_results.into_iter().map(Insight::Causal));
            },
        }
        
        Ok(insights)
    }
    
    pub async fn drill_down(&mut self, session: &mut ExplorationSession, drill_config: DrillDownConfig) -> Result<DrillDownResult, AnalysisError> {
        // Apply filters and create subset
        let filtered_data = self.filter_engine.apply_filters(&session.dataset, &drill_config.filters)?;
        
        // Perform focused analysis on subset
        let focused_analysis = self.analysis_engine.analyze_subset(
            &filtered_data,
            &drill_config.analysis_focus
        ).await?;
        
        // Create drill-down visualization
        let visualization = self.visualization_engine.create_drill_down_visualization(
            &filtered_data,
            &focused_analysis,
            &drill_config.visualization_config
        ).await?;
        
        // Update session history
        session.analysis_history.push(AnalysisStep {
            step_type: AnalysisStepType::DrillDown,
            timestamp: Utc::now(),
            config: serde_json::to_value(&drill_config)?,
            results: serde_json::to_value(&focused_analysis)?,
        });
        
        Ok(DrillDownResult {
            filtered_data,
            analysis: focused_analysis,
            visualization,
            insights: self.generate_drill_down_insights(&focused_analysis).await?,
        })
    }
    
    pub async fn find_correlations(&mut self, session: &ExplorationSession, correlation_config: CorrelationConfig) -> Result<CorrelationMatrix, AnalysisError> {
        self.correlation_analyzer.compute_correlation_matrix(
            &session.dataset,
            &correlation_config.variables,
            correlation_config.method,
            correlation_config.significance_level
        ).await
    }
    
    pub async fn detect_anomalies(&mut self, session: &ExplorationSession, anomaly_config: AnomalyDetectionConfig) -> Result<AnomalyReport, AnalysisError> {
        let anomalies = self.analysis_engine.detect_anomalies_advanced(
            &session.dataset,
            &anomaly_config
        ).await?;
        
        let report = AnomalyReport {
            anomalies,
            detection_method: anomaly_config.method,
            confidence_intervals: self.calculate_confidence_intervals(&session.dataset).await?,
            recommendations: self.generate_anomaly_recommendations(&anomalies).await?,
        };
        
        Ok(report)
    }
    
    pub async fn predict_trends(&mut self, session: &ExplorationSession, prediction_config: PredictionConfig) -> Result<TrendPrediction, AnalysisError> {
        // Prepare time series data
        let time_series = self.data_manager.extract_time_series(&session.dataset, &prediction_config.target_variable)?;
        
        // Apply forecasting model
        let forecast = match prediction_config.model_type {
            ForecastingModel::ARIMA { p, d, q } => {
                self.analysis_engine.forecast_arima(&time_series, p, d, q, prediction_config.forecast_horizon).await?
            },
            
            ForecastingModel::ExponentialSmoothing { alpha, beta, gamma } => {
                self.analysis_engine.forecast_exponential_smoothing(&time_series, alpha, beta, gamma, prediction_config.forecast_horizon).await?
            },
            
            ForecastingModel::NeuralNetwork { architecture } => {
                self.analysis_engine.forecast_neural_network(&time_series, &architecture, prediction_config.forecast_horizon).await?
            },
            
            ForecastingModel::Ensemble { models } => {
                self.analysis_engine.forecast_ensemble(&time_series, &models, prediction_config.forecast_horizon).await?
            },
        };
        
        Ok(TrendPrediction {
            forecast,
            confidence_intervals: self.calculate_prediction_intervals(&forecast, &prediction_config).await?,
            model_performance: self.evaluate_model_performance(&time_series, &forecast).await?,
            feature_importance: self.calculate_feature_importance(&session.dataset, &prediction_config).await?,
        })
    }
}

#[derive(Debug)]
pub struct CorrelationAnalyzer {
    correlation_cache: LruCache<String, CorrelationMatrix>,
    statistical_engine: StatisticalEngine,
}

impl CorrelationAnalyzer {
    pub fn new() -> Self {
        Self {
            correlation_cache: LruCache::new(100),
            statistical_engine: StatisticalEngine::new(),
        }
    }
    
    pub async fn compute_correlation_matrix(&mut self, dataset: &Dataset, variables: &[String], method: CorrelationMethod, significance_level: f64) -> Result<CorrelationMatrix, AnalysisError> {
        let cache_key = format!("{}_{:?}_{}", 
                               variables.join(","), 
                               method, 
                               significance_level);
        
        if let Some(cached_matrix) = self.correlation_cache.get(&cache_key) {
            return Ok(cached_matrix.clone());
        }
        
        let mut matrix = CorrelationMatrix::new(variables.len());
        
        for (i, var1) in variables.iter().enumerate() {
            for (j, var2) in variables.iter().enumerate() {
                if i <= j {
                    let correlation = self.calculate_correlation(dataset, var1, var2, method).await?;
                    let p_value = self.calculate_p_value(&correlation, dataset.len(), method).await?;
                    let is_significant = p_value < significance_level;
                    
                    matrix.set_correlation(i, j, CorrelationValue {
                        coefficient: correlation,
                        p_value,
                        is_significant,
                        sample_size: dataset.len(),
                    });
                }
            }
        }
        
        self.correlation_cache.put(cache_key, matrix.clone());
        Ok(matrix)
    }
    
    async fn calculate_correlation(&self, dataset: &Dataset, var1: &str, var2: &str, method: CorrelationMethod) -> Result<f64, AnalysisError> {
        let series1 = dataset.get_numeric_column(var1)?;
        let series2 = dataset.get_numeric_column(var2)?;
        
        match method {
            CorrelationMethod::Pearson => {
                self.statistical_engine.pearson_correlation(&series1, &series2)
            },
            CorrelationMethod::Spearman => {
                self.statistical_engine.spearman_correlation(&series1, &series2)
            },
            CorrelationMethod::Kendall => {
                self.statistical_engine.kendall_correlation(&series1, &series2)
            },
            CorrelationMethod::MutualInformation => {
                self.statistical_engine.mutual_information(&series1, &series2).await
            },
        }
    }
    
    pub async fn analyze_correlations(&mut self, dataset: &Dataset, threshold: f64) -> Result<Vec<CorrelationInsight>, AnalysisError> {
        let numeric_columns = dataset.get_numeric_column_names();
        let correlation_matrix = self.compute_correlation_matrix(
            dataset,
            &numeric_columns,
            CorrelationMethod::Pearson,
            0.05
        ).await?;
        
        let mut insights = Vec::new();
        
        // Find strong correlations
        for (i, var1) in numeric_columns.iter().enumerate() {
            for (j, var2) in numeric_columns.iter().enumerate() {
                if i < j {
                    let correlation = correlation_matrix.get_correlation(i, j);
                    if correlation.coefficient.abs() > threshold && correlation.is_significant {
                        insights.push(CorrelationInsight {
                            variable1: var1.clone(),
                            variable2: var2.clone(),
                            correlation_coefficient: correlation.coefficient,
                            strength: self.classify_correlation_strength(correlation.coefficient.abs()),
                            direction: if correlation.coefficient > 0.0 { CorrelationDirection::Positive } else { CorrelationDirection::Negative },
                            p_value: correlation.p_value,
                            interpretation: self.generate_correlation_interpretation(var1, var2, &correlation),
                        });
                    }
                }
            }
        }
        
        // Sort by correlation strength
        insights.sort_by(|a, b| b.correlation_coefficient.abs().partial_cmp(&a.correlation_coefficient.abs()).unwrap_or(std::cmp::Ordering::Equal));
        
        Ok(insights)
    }
}
```

#### 2. Statistical Analysis Suite
```rust
// src/analysis/statistical_suite.rs
#[derive(Debug)]
pub struct StatisticalAnalysisSuite {
    hypothesis_tester: HypothesisTester,
    distribution_analyzer: DistributionAnalyzer,
    regression_analyzer: RegressionAnalyzer,
    time_series_analyzer: TimeSeriesAnalyzer,
    bayesian_analyzer: BayesianAnalyzer,
    bootstrap_engine: BootstrapEngine,
}

impl StatisticalAnalysisSuite {
    pub fn new() -> Self {
        Self {
            hypothesis_tester: HypothesisTester::new(),
            distribution_analyzer: DistributionAnalyzer::new(),
            regression_analyzer: RegressionAnalyzer::new(),
            time_series_analyzer: TimeSeriesAnalyzer::new(),
            bayesian_analyzer: BayesianAnalyzer::new(),
            bootstrap_engine: BootstrapEngine::new(),
        }
    }
    
    pub async fn perform_hypothesis_test(&mut self, dataset: &Dataset, test_config: HypothesisTestConfig) -> Result<HypothesisTestResult, AnalysisError> {
        match test_config.test_type {
            HypothesisTestType::TTest { sample_type, alternative } => {
                self.hypothesis_tester.t_test(dataset, &test_config, sample_type, alternative).await
            },
            
            HypothesisTestType::ChiSquare { expected_frequencies } => {
                self.hypothesis_tester.chi_square_test(dataset, &test_config, expected_frequencies).await
            },
            
            HypothesisTestType::ANOVA { groups } => {
                self.hypothesis_tester.anova_test(dataset, &test_config, groups).await
            },
            
            HypothesisTestType::KolmogorovSmirnov { reference_distribution } => {
                self.hypothesis_tester.ks_test(dataset, &test_config, reference_distribution).await
            },
            
            HypothesisTestType::MannWhitneyU => {
                self.hypothesis_tester.mann_whitney_u_test(dataset, &test_config).await
            },
            
            HypothesisTestType::WilcoxonSignedRank => {
                self.hypothesis_tester.wilcoxon_signed_rank_test(dataset, &test_config).await
            },
        }
    }
    
    pub async fn analyze_distribution(&mut self, dataset: &Dataset, variable: &str) -> Result<DistributionAnalysis, AnalysisError> {
        let data = dataset.get_numeric_column(variable)?;
        
        // Fit multiple distributions and compare
        let candidate_distributions = vec![
            DistributionType::Normal,
            DistributionType::LogNormal,
            DistributionType::Exponential,
            DistributionType::Gamma,
            DistributionType::Beta,
            DistributionType::Weibull,
        ];
        
        let mut fitted_distributions = Vec::new();
        
        for dist_type in candidate_distributions {
            let fitted = self.distribution_analyzer.fit_distribution(&data, dist_type).await?;
            fitted_distributions.push(fitted);
        }
        
        // Select best fit using AIC/BIC
        let best_fit = fitted_distributions.into_iter()
            .min_by(|a, b| a.aic.partial_cmp(&b.aic).unwrap_or(std::cmp::Ordering::Equal))
            .ok_or(AnalysisError::DistributionFittingFailed)?;
        
        // Generate distribution statistics
        let statistics = DistributionStatistics {
            mean: self.calculate_mean(&data),
            median: self.calculate_median(&data),
            mode: self.calculate_mode(&data),
            variance: self.calculate_variance(&data),
            skewness: self.calculate_skewness(&data),
            kurtosis: self.calculate_kurtosis(&data),
            quantiles: self.calculate_quantiles(&data, vec![0.25, 0.5, 0.75, 0.9, 0.95, 0.99]),
        };
        
        // Perform normality tests
        let normality_tests = vec![
            self.hypothesis_tester.shapiro_wilk_test(&data).await?,
            self.hypothesis_tester.anderson_darling_test(&data).await?,
            self.hypothesis_tester.jarque_bera_test(&data).await?,
        ];
        
        Ok(DistributionAnalysis {
            variable: variable.to_string(),
            sample_size: data.len(),
            statistics,
            best_fit_distribution: best_fit,
            normality_tests,
            outlier_analysis: self.analyze_outliers(&data).await?,
            visualization_data: self.generate_distribution_plots(&data).await?,
        })
    }
    
    pub async fn perform_regression_analysis(&mut self, dataset: &Dataset, regression_config: RegressionConfig) -> Result<RegressionAnalysis, AnalysisError> {
        match regression_config.model_type {
            RegressionType::Linear => {
                self.regression_analyzer.linear_regression(dataset, &regression_config).await
            },
            
            RegressionType::Polynomial { degree } => {
                self.regression_analyzer.polynomial_regression(dataset, &regression_config, degree).await
            },
            
            RegressionType::Logistic => {
                self.regression_analyzer.logistic_regression(dataset, &regression_config).await
            },
            
            RegressionType::Ridge { alpha } => {
                self.regression_analyzer.ridge_regression(dataset, &regression_config, alpha).await
            },
            
            RegressionType::Lasso { alpha } => {
                self.regression_analyzer.lasso_regression(dataset, &regression_config, alpha).await
            },
            
            RegressionType::ElasticNet { alpha, l1_ratio } => {
                self.regression_analyzer.elastic_net_regression(dataset, &regression_config, alpha, l1_ratio).await
            },
        }
    }
    
    pub async fn analyze_time_series(&mut self, dataset: &Dataset, time_series_config: TimeSeriesConfig) -> Result<TimeSeriesAnalysis, AnalysisError> {
        let time_series = dataset.get_time_series(&time_series_config.timestamp_column, &time_series_config.value_column)?;
        
        // Decompose time series
        let decomposition = self.time_series_analyzer.decompose_time_series(&time_series, time_series_config.seasonality_period).await?;
        
        // Test for stationarity
        let stationarity_tests = vec![
            self.time_series_analyzer.augmented_dickey_fuller_test(&time_series).await?,
            self.time_series_analyzer.kpss_test(&time_series).await?,
            self.time_series_analyzer.phillips_perron_test(&time_series).await?,
        ];
        
        // Detect change points
        let change_points = self.time_series_analyzer.detect_change_points(&time_series, time_series_config.change_point_method).await?;
        
        // Analyze seasonality
        let seasonality_analysis = self.time_series_analyzer.analyze_seasonality(&time_series, time_series_config.seasonality_period).await?;
        
        // Fit ARIMA model
        let arima_model = self.time_series_analyzer.fit_arima_model(&time_series, time_series_config.arima_config).await?;
        
        Ok(TimeSeriesAnalysis {
            time_series_length: time_series.len(),
            decomposition,
            stationarity_tests,
            change_points,
            seasonality_analysis,
            trend_analysis: self.analyze_trend(&decomposition.trend).await?,
            arima_model,
            forecast: self.time_series_analyzer.forecast(&arima_model, time_series_config.forecast_horizon).await?,
            residual_analysis: self.analyze_residuals(&arima_model.residuals).await?,
        })
    }
    
    pub async fn perform_bayesian_analysis(&mut self, dataset: &Dataset, bayesian_config: BayesianConfig) -> Result<BayesianAnalysis, AnalysisError> {
        match bayesian_config.analysis_type {
            BayesianAnalysisType::ParameterEstimation { prior_distribution, likelihood } => {
                self.bayesian_analyzer.estimate_parameters(dataset, &bayesian_config, prior_distribution, likelihood).await
            },
            
            BayesianAnalysisType::ModelComparison { models } => {
                self.bayesian_analyzer.compare_models(dataset, &bayesian_config, models).await
            },
            
            BayesianAnalysisType::HypothesisTesting { null_hypothesis, alternative_hypothesis } => {
                self.bayesian_analyzer.test_hypothesis(dataset, &bayesian_config, null_hypothesis, alternative_hypothesis).await
            },
            
            BayesianAnalysisType::PredictivePosterior { prediction_points } => {
                self.bayesian_analyzer.compute_predictive_posterior(dataset, &bayesian_config, prediction_points).await
            },
        }
    }
}

#[derive(Debug)]
pub struct HypothesisTester {
    significance_level: f64,
    power_analysis_engine: PowerAnalysisEngine,
}

impl HypothesisTester {
    pub fn new() -> Self {
        Self {
            significance_level: 0.05,
            power_analysis_engine: PowerAnalysisEngine::new(),
        }
    }
    
    pub async fn t_test(&mut self, dataset: &Dataset, config: &HypothesisTestConfig, sample_type: TTestType, alternative: AlternativeHypothesis) -> Result<HypothesisTestResult, AnalysisError> {
        let data = dataset.get_numeric_column(&config.variable)?;
        
        let (statistic, p_value, degrees_of_freedom) = match sample_type {
            TTestType::OneSample { population_mean } => {
                self.one_sample_t_test(&data, population_mean, alternative)
            },
            
            TTestType::TwoSampleIndependent { group_variable } => {
                let group_data = dataset.get_categorical_column(&group_variable)?;
                self.two_sample_independent_t_test(&data, &group_data, alternative)?
            },
            
            TTestType::TwoSamplePaired { paired_variable } => {
                let paired_data = dataset.get_numeric_column(&paired_variable)?;
                self.paired_t_test(&data, &paired_data, alternative)
            },
        };
        
        // Calculate effect size (Cohen's d)
        let effect_size = self.calculate_cohens_d(&data, &sample_type, dataset)?;
        
        // Perform power analysis
        let power_analysis = self.power_analysis_engine.analyze_t_test_power(
            effect_size,
            data.len(),
            self.significance_level,
            alternative
        ).await?;
        
        // Generate confidence interval
        let confidence_interval = self.calculate_t_test_confidence_interval(&data, &sample_type, 1.0 - self.significance_level)?;
        
        Ok(HypothesisTestResult {
            test_type: format!("{:?}", sample_type),
            null_hypothesis: config.null_hypothesis.clone(),
            alternative_hypothesis: format!("{:?}", alternative),
            test_statistic: statistic,
            p_value,
            degrees_of_freedom: Some(degrees_of_freedom),
            is_significant: p_value < self.significance_level,
            effect_size: Some(effect_size),
            confidence_interval: Some(confidence_interval),
            power_analysis: Some(power_analysis),
            interpretation: self.generate_t_test_interpretation(statistic, p_value, effect_size, alternative),
        })
    }
    
    fn one_sample_t_test(&self, data: &[f64], population_mean: f64, alternative: AlternativeHypothesis) -> (f64, f64, f64) {
        let n = data.len() as f64;
        let sample_mean = data.iter().sum::<f64>() / n;
        let sample_std = self.calculate_sample_standard_deviation(data);
        
        let t_statistic = (sample_mean - population_mean) / (sample_std / n.sqrt());
        let degrees_of_freedom = n - 1.0;
        
        let p_value = match alternative {
            AlternativeHypothesis::TwoTailed => 2.0 * (1.0 - self.t_distribution_cdf(t_statistic.abs(), degrees_of_freedom)),
            AlternativeHypothesis::Greater => 1.0 - self.t_distribution_cdf(t_statistic, degrees_of_freedom),
            AlternativeHypothesis::Less => self.t_distribution_cdf(t_statistic, degrees_of_freedom),
        };
        
        (t_statistic, p_value, degrees_of_freedom)
    }
    
    fn two_sample_independent_t_test(&self, data: &[f64], groups: &[String], alternative: AlternativeHypothesis) -> Result<(f64, f64, f64), AnalysisError> {
        // Separate data by groups
        let unique_groups: Vec<&String> = groups.iter().collect::<std::collections::HashSet<_>>().into_iter().collect();
        if unique_groups.len() != 2 {
            return Err(AnalysisError::InvalidGroupCount(unique_groups.len()));
        }
        
        let group1_data: Vec<f64> = data.iter().zip(groups.iter())
            .filter(|(_, group)| group == &unique_groups[0])
            .map(|(value, _)| *value)
            .collect();
        
        let group2_data: Vec<f64> = data.iter().zip(groups.iter())
            .filter(|(_, group)| group == &unique_groups[1])
            .map(|(value, _)| *value)
            .collect();
        
        let n1 = group1_data.len() as f64;
        let n2 = group2_data.len() as f64;
        let mean1 = group1_data.iter().sum::<f64>() / n1;
        let mean2 = group2_data.iter().sum::<f64>() / n2;
        let var1 = self.calculate_sample_variance(&group1_data);
        let var2 = self.calculate_sample_variance(&group2_data);
        
        // Welch's t-test (unequal variances)
        let pooled_se = (var1 / n1 + var2 / n2).sqrt();
        let t_statistic = (mean1 - mean2) / pooled_se;
        
        // Welch-Satterthwaite equation for degrees of freedom
        let degrees_of_freedom = (var1 / n1 + var2 / n2).powi(2) / 
            ((var1 / n1).powi(2) / (n1 - 1.0) + (var2 / n2).powi(2) / (n2 - 1.0));
        
        let p_value = match alternative {
            AlternativeHypothesis::TwoTailed => 2.0 * (1.0 - self.t_distribution_cdf(t_statistic.abs(), degrees_of_freedom)),
            AlternativeHypothesis::Greater => 1.0 - self.t_distribution_cdf(t_statistic, degrees_of_freedom),
            AlternativeHypothesis::Less => self.t_distribution_cdf(t_statistic, degrees_of_freedom),
        };
        
        Ok((t_statistic, p_value, degrees_of_freedom))
    }
}
```

#### 3. Machine Learning Analysis Tools
```rust
// src/analysis/ml_tools.rs
#[derive(Debug)]
pub struct MachineLearningAnalysisTools {
    anomaly_detector: MLAnomalyDetector,
    predictive_modeler: PredictiveModeler,
    clustering_engine: ClusteringEngine,
    feature_analyzer: FeatureAnalyzer,
    model_validator: ModelValidator,
    automl_engine: AutoMLEngine,
}

impl MachineLearningAnalysisTools {
    pub fn new() -> Self {
        Self {
            anomaly_detector: MLAnomalyDetector::new(),
            predictive_modeler: PredictiveModeler::new(),
            clustering_engine: ClusteringEngine::new(),
            feature_analyzer: FeatureAnalyzer::new(),
            model_validator: ModelValidator::new(),
            automl_engine: AutoMLEngine::new(),
        }
    }
    
    pub async fn detect_anomalies_ml(&mut self, dataset: &Dataset, config: MLAnomalyConfig) -> Result<MLAnomalyReport, AnalysisError> {
        // Prepare features
        let features = self.feature_analyzer.extract_features(dataset, &config.feature_config).await?;
        
        // Apply multiple anomaly detection algorithms
        let mut detection_results = Vec::new();
        
        // Isolation Forest
        if config.algorithms.contains(&AnomalyAlgorithm::IsolationForest) {
            let isolation_forest_result = self.anomaly_detector.isolation_forest(&features, &config.isolation_forest_config).await?;
            detection_results.push(("Isolation Forest".to_string(), isolation_forest_result));
        }
        
        // Local Outlier Factor
        if config.algorithms.contains(&AnomalyAlgorithm::LocalOutlierFactor) {
            let lof_result = self.anomaly_detector.local_outlier_factor(&features, &config.lof_config).await?;
            detection_results.push(("Local Outlier Factor".to_string(), lof_result));
        }
        
        // One-Class SVM
        if config.algorithms.contains(&AnomalyAlgorithm::OneClassSVM) {
            let svm_result = self.anomaly_detector.one_class_svm(&features, &config.svm_config).await?;
            detection_results.push(("One-Class SVM".to_string(), svm_result));
        }
        
        // Autoencoder
        if config.algorithms.contains(&AnomalyAlgorithm::Autoencoder) {
            let autoencoder_result = self.anomaly_detector.autoencoder_anomaly_detection(&features, &config.autoencoder_config).await?;
            detection_results.push(("Autoencoder".to_string(), autoencoder_result));
        }
        
        // Ensemble results
        let ensemble_result = self.anomaly_detector.ensemble_anomaly_detection(&detection_results, &config.ensemble_config).await?;
        
        // Generate anomaly explanations
        let explanations = self.generate_anomaly_explanations(&features, &ensemble_result.anomalies, &config).await?;
        
        Ok(MLAnomalyReport {
            algorithm_results: detection_results,
            ensemble_result,
            anomaly_explanations: explanations,
            feature_importance: self.calculate_anomaly_feature_importance(&features, &ensemble_result.anomalies).await?,
            recommendations: self.generate_anomaly_recommendations(&ensemble_result.anomalies).await?,
        })
    }
    
    pub async fn build_predictive_model(&mut self, dataset: &Dataset, config: PredictiveModelConfig) -> Result<PredictiveModel, AnalysisError> {
        // Feature engineering
        let engineered_features = self.feature_analyzer.engineer_features(dataset, &config.feature_engineering_config).await?;
        
        // Split data
        let (train_data, test_data) = self.split_data(&engineered_features, config.train_test_split_ratio)?;
        
        // Model selection and hyperparameter tuning
        let best_model = if config.use_automl {
            self.automl_engine.auto_model_selection(&train_data, &config.target_variable, &config.automl_config).await?
        } else {
            self.train_specified_model(&train_data, &config.model_type, &config.hyperparameters).await?
        };
        
        // Model validation
        let validation_results = self.model_validator.cross_validate(&best_model, &train_data, config.cv_folds).await?;
        
        // Final model evaluation on test set
        let test_metrics = self.model_validator.evaluate_on_test_set(&best_model, &test_data).await?;
        
        // Feature importance analysis
        let feature_importance = self.calculate_feature_importance(&best_model, &engineered_features).await?;
        
        // Model interpretation
        let model_interpretation = self.generate_model_interpretation(&best_model, &engineered_features, &config).await?;
        
        Ok(PredictiveModel {
            model: best_model,
            training_metrics: validation_results,
            test_metrics,
            feature_importance,
            model_interpretation,
            feature_engineering_pipeline: engineered_features.pipeline,
            training_data_summary: self.summarize_training_data(&train_data),
        })
    }
    
    pub async fn perform_clustering_analysis(&mut self, dataset: &Dataset, config: ClusteringConfig) -> Result<ClusteringAnalysis, AnalysisError> {
        // Prepare features for clustering
        let features = self.feature_analyzer.prepare_clustering_features(dataset, &config.feature_config).await?;
        
        // Determine optimal number of clusters if not specified
        let optimal_clusters = if let Some(k) = config.num_clusters {
            k
        } else {
            self.clustering_engine.determine_optimal_clusters(&features, &config.cluster_selection_config).await?
        };
        
        // Apply clustering algorithm
        let clustering_result = match config.algorithm {
            ClusteringAlgorithm::KMeans => {
                self.clustering_engine.k_means_clustering(&features, optimal_clusters, &config.kmeans_config).await?
            },
            
            ClusteringAlgorithm::DBSCAN => {
                self.clustering_engine.dbscan_clustering(&features, &config.dbscan_config).await?
            },
            
            ClusteringAlgorithm::HierarchicalClustering => {
                self.clustering_engine.hierarchical_clustering(&features, optimal_clusters, &config.hierarchical_config).await?
            },
            
            ClusteringAlgorithm::GaussianMixture => {
                self.clustering_engine.gaussian_mixture_clustering(&features, optimal_clusters, &config.gmm_config).await?
            },
        };
        
        // Cluster validation
        let validation_metrics = self.clustering_engine.validate_clustering(&features, &clustering_result).await?;
        
        // Cluster profiling
        let cluster_profiles = self.generate_cluster_profiles(&features, &clustering_result, dataset).await?;
        
        // Dimensionality reduction for visualization
        let visualization_data = self.clustering_engine.create_cluster_visualization(&features, &clustering_result).await?;
        
        Ok(ClusteringAnalysis {
            clustering_result,
            validation_metrics,
            cluster_profiles,
            visualization_data,
            optimal_clusters_analysis: self.analyze_cluster_selection(&features, &config.cluster_selection_config).await?,
            cluster_stability: self.assess_cluster_stability(&features, &clustering_result).await?,
        })
    }
    
    async fn generate_anomaly_explanations(&self, features: &FeatureMatrix, anomalies: &[AnomalyInstance], config: &MLAnomalyConfig) -> Result<Vec<AnomalyExplanation>, AnalysisError> {
        let mut explanations = Vec::new();
        
        for anomaly in anomalies {
            let explanation = match config.explanation_method {
                ExplanationMethod::SHAP => {
                    self.anomaly_detector.explain_with_shap(features, anomaly).await?
                },
                
                ExplanationMethod::LIME => {
                    self.anomaly_detector.explain_with_lime(features, anomaly).await?
                },
                
                ExplanationMethod::FeatureContribution => {
                    self.anomaly_detector.explain_with_feature_contribution(features, anomaly).await?
                },
            };
            
            explanations.push(explanation);
        }
        
        Ok(explanations)
    }
    
    async fn train_specified_model(&mut self, train_data: &TrainingData, model_type: &ModelType, hyperparameters: &HashMap<String, serde_json::Value>) -> Result<TrainedModel, AnalysisError> {
        match model_type {
            ModelType::LinearRegression => {
                self.predictive_modeler.train_linear_regression(train_data, hyperparameters).await
            },
            
            ModelType::RandomForest => {
                self.predictive_modeler.train_random_forest(train_data, hyperparameters).await
            },
            
            ModelType::GradientBoosting => {
                self.predictive_modeler.train_gradient_boosting(train_data, hyperparameters).await
            },
            
            ModelType::NeuralNetwork => {
                self.predictive_modeler.train_neural_network(train_data, hyperparameters).await
            },
            
            ModelType::SupportVectorMachine => {
                self.predictive_modeler.train_svm(train_data, hyperparameters).await
            },
        }
    }
}

#[derive(Debug)]
pub struct AutoMLEngine {
    model_search_space: ModelSearchSpace,
    hyperparameter_optimizer: HyperparameterOptimizer,
    feature_selector: AutoFeatureSelector,
    pipeline_optimizer: PipelineOptimizer,
}

impl AutoMLEngine {
    pub fn new() -> Self {
        Self {
            model_search_space: ModelSearchSpace::default(),
            hyperparameter_optimizer: HyperparameterOptimizer::new(),
            feature_selector: AutoFeatureSelector::new(),
            pipeline_optimizer: PipelineOptimizer::new(),
        }
    }
    
    pub async fn auto_model_selection(&mut self, train_data: &TrainingData, target_variable: &str, config: &AutoMLConfig) -> Result<TrainedModel, AnalysisError> {
        // Automatic feature selection
        let selected_features = self.feature_selector.select_features(train_data, target_variable, &config.feature_selection_config).await?;
        
        // Model search
        let mut best_model = None;
        let mut best_score = f64::NEG_INFINITY;
        
        for model_type in &self.model_search_space.models {
            // Hyperparameter optimization for each model type
            let optimized_model = self.hyperparameter_optimizer.optimize(
                model_type,
                &selected_features,
                target_variable,
                &config.optimization_config
            ).await?;
            
            // Evaluate model
            let score = self.evaluate_model(&optimized_model, &selected_features, &config.evaluation_metric).await?;
            
            if score > best_score {
                best_score = score;
                best_model = Some(optimized_model);
            }
        }
        
        let final_model = best_model.ok_or(AnalysisError::AutoMLFailed("No suitable model found".to_string()))?;
        
        // Pipeline optimization
        let optimized_pipeline = self.pipeline_optimizer.optimize_pipeline(&final_model, &selected_features).await?;
        
        Ok(TrainedModel {
            model: final_model,
            pipeline: optimized_pipeline,
            selected_features,
            training_score: best_score,
            hyperparameters: self.extract_hyperparameters(&final_model),
        })
    }
}
```

#### 4. Automated Insight Generation
```rust
// src/analysis/insight_generation.rs
#[derive(Debug)]
pub struct AutomatedInsightGenerator {
    pattern_recognizer: PatternRecognizer,
    narrative_generator: NarrativeGenerator,
    significance_assessor: SignificanceAssessor,
    recommendation_engine: RecommendationEngine,
    insight_ranker: InsightRanker,
}

impl AutomatedInsightGenerator {
    pub fn new() -> Self {
        Self {
            pattern_recognizer: PatternRecognizer::new(),
            narrative_generator: NarrativeGenerator::new(),
            significance_assessor: SignificanceAssessor::new(),
            recommendation_engine: RecommendationEngine::new(),
            insight_ranker: InsightRanker::new(),
        }
    }
    
    pub async fn generate_insights(&mut self, analysis_results: &AnalysisResults, context: &AnalysisContext) -> Result<Vec<GeneratedInsight>, AnalysisError> {
        let mut insights = Vec::new();
        
        // Pattern-based insights
        let patterns = self.pattern_recognizer.identify_patterns(analysis_results).await?;
        for pattern in patterns {
            let insight = self.create_pattern_insight(pattern, context).await?;
            insights.push(insight);
        }
        
        // Statistical insights
        let statistical_insights = self.generate_statistical_insights(analysis_results, context).await?;
        insights.extend(statistical_insights);
        
        // Comparative insights
        let comparative_insights = self.generate_comparative_insights(analysis_results, context).await?;
        insights.extend(comparative_insights);
        
        // Predictive insights
        let predictive_insights = self.generate_predictive_insights(analysis_results, context).await?;
        insights.extend(predictive_insights);
        
        // Assess significance and rank insights
        for insight in &mut insights {
            insight.significance_score = self.significance_assessor.assess_significance(&insight, context).await?;
            insight.confidence_level = self.calculate_confidence_level(&insight, analysis_results).await?;
        }
        
        // Rank insights by importance
        self.insight_ranker.rank_insights(&mut insights, context).await?;
        
        // Generate natural language explanations
        for insight in &mut insights {
            insight.explanation = self.narrative_generator.generate_explanation(&insight, context).await?;
            insight.recommendations = self.recommendation_engine.generate_recommendations(&insight, context).await?;
        }
        
        Ok(insights)
    }
    
    async fn generate_statistical_insights(&mut self, results: &AnalysisResults, context: &AnalysisContext) -> Result<Vec<GeneratedInsight>, AnalysisError> {
        let mut insights = Vec::new();
        
        // Distribution insights
        if let Some(distribution_analysis) = &results.distribution_analysis {
            insights.extend(self.extract_distribution_insights(distribution_analysis, context).await?);
        }
        
        // Correlation insights
        if let Some(correlation_matrix) = &results.correlation_analysis {
            insights.extend(self.extract_correlation_insights(correlation_matrix, context).await?);
        }
        
        // Hypothesis test insights
        if let Some(hypothesis_tests) = &results.hypothesis_tests {
            insights.extend(self.extract_hypothesis_insights(hypothesis_tests, context).await?);
        }
        
        // Regression insights
        if let Some(regression_analysis) = &results.regression_analysis {
            insights.extend(self.extract_regression_insights(regression_analysis, context).await?);
        }
        
        Ok(insights)
    }
    
    async fn extract_distribution_insights(&self, analysis: &DistributionAnalysis, context: &AnalysisContext) -> Result<Vec<GeneratedInsight>, AnalysisError> {
        let mut insights = Vec::new();
        
        // Normality insight
        let normality_p_values: Vec<f64> = analysis.normality_tests.iter().map(|test| test.p_value).collect();
        let avg_p_value = normality_p_values.iter().sum::<f64>() / normality_p_values.len() as f64;
        
        if avg_p_value < 0.05 {
            insights.push(GeneratedInsight {
                insight_type: InsightType::DistributionShape,
                title: format!("{} is not normally distributed", analysis.variable),
                description: format!(
                    "The variable {} shows significant deviation from normality (average p-value: {:.4}). \
                     This suggests the data has {} distribution characteristics.",
                    analysis.variable,
                    avg_p_value,
                    analysis.best_fit_distribution.distribution_type.name()
                ),
                evidence: self.create_normality_evidence(&analysis.normality_tests),
                significance_score: 0.0, // Will be calculated later
                confidence_level: 0.0,   // Will be calculated later
                explanation: String::new(), // Will be generated later
                recommendations: Vec::new(), // Will be generated later
                metadata: self.create_distribution_metadata(analysis),
            });
        }
        
        // Outlier insights
        if !analysis.outlier_analysis.outliers.is_empty() {
            let outlier_percentage = (analysis.outlier_analysis.outliers.len() as f64 / analysis.sample_size as f64) * 100.0;
            
            insights.push(GeneratedInsight {
                insight_type: InsightType::Outliers,
                title: format!("{} contains {:.1}% outliers", analysis.variable, outlier_percentage),
                description: format!(
                    "Detected {} outliers in {} ({:.1}% of data points). \
                     These outliers may indicate data quality issues or represent interesting edge cases.",
                    analysis.outlier_analysis.outliers.len(),
                    analysis.variable,
                    outlier_percentage
                ),
                evidence: self.create_outlier_evidence(&analysis.outlier_analysis),
                significance_score: 0.0,
                confidence_level: 0.0,
                explanation: String::new(),
                recommendations: Vec::new(),
                metadata: self.create_outlier_metadata(&analysis.outlier_analysis),
            });
        }
        
        // Skewness insight
        let skewness = analysis.statistics.skewness;
        if skewness.abs() > 1.0 {
            let skew_direction = if skewness > 0.0 { "right" } else { "left" };
            
            insights.push(GeneratedInsight {
                insight_type: InsightType::DistributionShape,
                title: format!("{} is highly skewed to the {}", analysis.variable, skew_direction),
                description: format!(
                    "The variable {} shows high {} skewness (skewness = {:.2}). \
                     This indicates the distribution has a long tail on the {} side.",
                    analysis.variable,
                    skew_direction,
                    skewness,
                    skew_direction
                ),
                evidence: self.create_skewness_evidence(skewness),
                significance_score: 0.0,
                confidence_level: 0.0,
                explanation: String::new(),
                recommendations: Vec::new(),
                metadata: self.create_skewness_metadata(skewness),
            });
        }
        
        Ok(insights)
    }
    
    async fn extract_correlation_insights(&self, correlation_matrix: &CorrelationMatrix, context: &AnalysisContext) -> Result<Vec<GeneratedInsight>, AnalysisError> {
        let mut insights = Vec::new();
        
        // Strong correlation insights
        let strong_correlations = correlation_matrix.get_strong_correlations(0.7);
        
        for correlation in strong_correlations {
            let strength = if correlation.coefficient.abs() > 0.9 {
                "very strong"
            } else if correlation.coefficient.abs() > 0.7 {
                "strong"
            } else {
                "moderate"
            };
            
            let direction = if correlation.coefficient > 0.0 { "positive" } else { "negative" };
            
            insights.push(GeneratedInsight {
                insight_type: InsightType::Correlation,
                title: format!("{} correlation between {} and {}", 
                              strength.to_title_case(), 
                              correlation.variable1, 
                              correlation.variable2),
                description: format!(
                    "Found a {} {} correlation (r = {:.3}, p < {:.3}) between {} and {}. \
                     This suggests these variables {} together.",
                    strength,
                    direction,
                    correlation.coefficient,
                    correlation.p_value,
                    correlation.variable1,
                    correlation.variable2,
                    if correlation.coefficient > 0.0 { "increase" } else { "move oppositely" }
                ),
                evidence: self.create_correlation_evidence(&correlation),
                significance_score: 0.0,
                confidence_level: 0.0,
                explanation: String::new(),
                recommendations: Vec::new(),
                metadata: self.create_correlation_metadata(&correlation),
            });
        }
        
        Ok(insights)
    }
}

#[derive(Debug)]
pub struct NarrativeGenerator {
    template_engine: TemplateEngine,
    language_model: LanguageModel,
    domain_knowledge: DomainKnowledgeBase,
}

impl NarrativeGenerator {
    pub fn new() -> Self {
        Self {
            template_engine: TemplateEngine::new(),
            language_model: LanguageModel::new(),
            domain_knowledge: DomainKnowledgeBase::load_dora_domain(),
        }
    }
    
    pub async fn generate_explanation(&mut self, insight: &GeneratedInsight, context: &AnalysisContext) -> Result<String, AnalysisError> {
        let template = self.template_engine.get_template_for_insight_type(&insight.insight_type)?;
        
        let context_variables = self.create_context_variables(insight, context);
        let base_explanation = self.template_engine.render(&template, &context_variables)?;
        
        // Enhance with domain-specific knowledge
        let enhanced_explanation = self.domain_knowledge.enhance_explanation(&base_explanation, insight)?;
        
        // Apply natural language generation for readability
        let final_explanation = self.language_model.improve_readability(&enhanced_explanation).await?;
        
        Ok(final_explanation)
    }
    
    fn create_context_variables(&self, insight: &GeneratedInsight, context: &AnalysisContext) -> HashMap<String, String> {
        let mut variables = HashMap::new();
        
        variables.insert("insight_title".to_string(), insight.title.clone());
        variables.insert("insight_description".to_string(), insight.description.clone());
        variables.insert("significance_score".to_string(), format!("{:.2}", insight.significance_score));
        variables.insert("confidence_level".to_string(), format!("{:.1}%", insight.confidence_level * 100.0));
        
        // Add context-specific variables
        if let Some(dataflow_name) = &context.dataflow_name {
            variables.insert("dataflow_name".to_string(), dataflow_name.clone());
        }
        
        if let Some(time_range) = &context.time_range {
            variables.insert("analysis_period".to_string(), format!("{} to {}", time_range.start, time_range.end));
        }
        
        variables
    }
}
```

### Why This Approach

**Comprehensive Analysis Suite:**
- Interactive data exploration with drill-down capabilities
- Statistical analysis with hypothesis testing and distribution fitting
- Machine learning tools for anomaly detection and predictive modeling
- Automated insight generation with natural language explanations

**Advanced Analytics:**
- Multiple correlation methods and significance testing
- Time series analysis with decomposition and forecasting
- Bayesian analysis for parameter estimation and model comparison
- AutoML capabilities for automated model selection

**User-Friendly Interface:**
- Interactive visualizations with real-time updates
- Natural language explanations of findings
- Automated recommendations and actionable insights
- Comparative analysis for A/B testing and benchmarking

### How to Implement

#### Step 1: Data Explorer Core (6 hours)
1. **Implement InteractiveDataExplorer** with comprehensive analysis engine
2. **Add drill-down capabilities** with dynamic filtering and subset analysis
3. **Create correlation analyzer** with multiple correlation methods
4. **Add pattern detection** with automated discovery algorithms

#### Step 2: Statistical Analysis Suite (8 hours)
1. **Implement StatisticalAnalysisSuite** with hypothesis testing
2. **Add distribution analysis** with automatic fitting and goodness-of-fit tests
3. **Create regression analyzer** with multiple regression types
4. **Add time series analysis** with decomposition and stationarity testing

#### Step 3: Machine Learning Tools (7 hours)
1. **Implement MachineLearningAnalysisTools** with anomaly detection
2. **Add predictive modeling** with AutoML capabilities
3. **Create clustering analysis** with multiple algorithms and validation
4. **Add feature analysis** with automated selection and engineering

#### Step 4: Insight Generation (5 hours)
1. **Implement AutomatedInsightGenerator** with pattern recognition
2. **Add narrative generation** with natural language explanations
3. **Create recommendation engine** with actionable suggestions
4. **Add significance assessment** and insight ranking

#### Step 5: Integration and Testing (3 hours)
1. **Add comprehensive unit tests** for all analysis components
2. **Test statistical accuracy** and machine learning model performance
3. **Validate insight generation** quality and relevance
4. **Test performance** with large datasets and complex analyses

## 🔗 Dependencies
**Depends On:**
- Issue #032 (Advanced Data Visualization) - Visualization engine integration
- Issue #024 (Dashboard Overview) - Integration with main interface
- Phase 2 enhanced commands for data source access

**Enables:**
- Deep analytical insights into Dora system behavior
- Predictive capabilities for proactive system management
- Automated discovery of optimization opportunities

## 🧪 Testing Requirements

### Unit Tests
```rust
#[cfg(test)]
mod tests {
    use super::*;
    
    #[test]
    fn test_correlation_analysis_accuracy() {
        let dataset = create_test_dataset_with_known_correlations();
        let mut analyzer = CorrelationAnalyzer::new();
        
        let correlation_matrix = analyzer.compute_correlation_matrix(
            &dataset,
            &vec!["var1".to_string(), "var2".to_string()],
            CorrelationMethod::Pearson,
            0.05
        ).await.unwrap();
        
        let correlation = correlation_matrix.get_correlation(0, 1);
        assert!((correlation.coefficient - 0.85).abs() < 0.01); // Known correlation
        assert!(correlation.is_significant);
    }
    
    #[test]
    fn test_anomaly_detection_performance() {
        let mut ml_tools = MachineLearningAnalysisTools::new();
        let dataset_with_anomalies = create_dataset_with_known_anomalies();
        
        let start_time = Instant::now();
        let anomaly_report = ml_tools.detect_anomalies_ml(&dataset_with_anomalies, MLAnomalyConfig::default()).await.unwrap();
        let detection_time = start_time.elapsed();
        
        assert!(detection_time < Duration::from_secs(2));
        assert!(anomaly_report.ensemble_result.precision > 0.85);
        assert!(anomaly_report.ensemble_result.recall > 0.80);
    }
    
    #[test]
    fn test_insight_generation_quality() {
        let mut generator = AutomatedInsightGenerator::new();
        let analysis_results = create_test_analysis_results();
        let context = AnalysisContext::test_context();
        
        let insights = generator.generate_insights(&analysis_results, &context).await.unwrap();
        
        assert!(!insights.is_empty());
        assert!(insights.iter().all(|insight| !insight.explanation.is_empty()));
        assert!(insights.iter().all(|insight| insight.significance_score > 0.0));
    }
}
```

## ✅ Definition of Done
- [ ] InteractiveDataExplorer provides comprehensive data exploration capabilities
- [ ] Statistical analysis suite supports all major statistical tests and methods
- [ ] Machine learning tools enable anomaly detection and predictive modeling
- [ ] Automated insight generation produces meaningful explanations and recommendations
- [ ] Performance targets met for all analysis operations on large datasets
- [ ] Correlation analysis provides accurate results with significance testing
- [ ] Anomaly detection achieves target accuracy rates across different data types
- [ ] Predictive models meet accuracy requirements for system behavior forecasting
- [ ] Natural language explanations are clear and actionable
- [ ] Drill-down and comparative analysis features work seamlessly
- [ ] Comprehensive unit tests validate all analysis functionality
- [ ] Integration tests confirm accuracy and performance of analysis tools
- [ ] Manual testing validates user experience and insight quality

This comprehensive interactive analysis tools suite provides developers with powerful capabilities for exploring, understanding, and predicting Dora system behavior through advanced statistical analysis, machine learning, and automated insight generation.