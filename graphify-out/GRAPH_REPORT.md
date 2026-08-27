# Graph Report - .  (2026-08-24)

## Corpus Check
- 403 files · ~339,030 words
- Verdict: corpus is large enough that graph structure adds value.

## Summary
- 4299 nodes · 7852 edges · 211 communities (207 shown, 4 thin omitted)
- Extraction: 95% EXTRACTED · 5% INFERRED · 0% AMBIGUOUS · INFERRED: 428 edges (avg confidence: 0.78)
- Token cost: 0 input · 0 output

## Community Hubs (Navigation)
- IMU Translation Probes
- VIO State Analysis
- Keyframe Schedule Probes
- EuRoC Dataset Tests
- VIO Initialization Probes
- Stereo Calibration Geometry
- Gyro Bias Alignment
- Benchmark CLI Configuration
- VIO State Probe Runtime
- Camera Model Tests
- Stereo VO Estimator
- M3.3 Benchmark Evidence
- M3.3 Slice Plans
- Estimator Diagnostics Types
- IMU Factor Construction
- Probe B Artifacts
- Stereo IMU Synchronizer
- Keyframe Selection Logic
- Sensor Frame Contracts
- Architecture Contracts
- Synthetic IMU Tests
- Camera Parameter Models
- Shadow Probe Records
- TUM VI Dataset Tests
- M3.3 Algorithm Designs
- VIO Diagnostic State
- VIO Bias Analysis
- Camera Projection API
- Candidate VIO Pipeline
- Trajectory Data Model
- Offline VO Session
- Relative Pose Evaluation
- Fixed Lag Shadow Analysis
- Dataset Replay Tests
- Candidate Frame Diagnostics
- Keyframe Epoch Gate
- Outlier Reoptimization Tests
- Gyro Visual Tests
- PnP Estimator Tests
- Stereo Rectifier
- Candidate Pipeline Tests
- VO Diagnostic Rows
- Gyro Graph Cost Diagnostics
- Reanchor Estimator Tests
- Candidate Fixed Lag Estimator
- Candidate Pipeline Runtime
- Fixed Lag Shadow Probe
- Keyframe Evidence Types
- Absolute Trajectory Evaluation
- Synchronizer Unit Tests
- Outlier Cull Tests
- Stereo Tracker Tests
- Synchronizer Diagnostics Contracts
- Benchmark Configuration Tests
- Benchmark Run Identity Tests
- Gyro Objective Analysis
- EuRoC Inspection CLI
- Pose Transform Tests
- Timestamp Association
- Keyframe Shadow Tests
- Candidate Probe Tests
- Session Frame Counters
- Estimator Measurement Types
- Benchmark Run Summary
- Estimator Diagnostics Tests
- Multi Frame Triangulation
- Estimator Type Tests
- Fixed Lag Diagnostics
- Stereo Tracker Options
- Dataset Internal Storage
- Offline Session Options
- Stereo Pair Stream
- M2 M3 Design Research
- M4 VIO Diagnosis Research
- Groundtruth Dataset Tests
- TUM Trajectory IO Tests
- Dataset Adapter Parsers
- Sensor Parameter Tests
- Candidate Artifact Writers
- Candidate Fixed Lag Runtime
- vio init probe
- TEST Cluster
- Ate Report
- create Cluster
- phad stereo frontend probe
- phad vo bench
- TEST Cluster
- stereo tracker
- stereo imu dataset
- fixed lag shadow probe test
- Timestamp Cluster
- Imu Measurement
- Project Working Agreements
- phad traj eval
- TEST Cluster
- query Git Identity
- Offline Analysis Scripts
- M3 3 VO Hardening Design
- update Fixed Lag Shadow
- Scene Cluster
- Robustness Summary
- stereo vo estimator
- Image Cluster
- vio init probe test
- bench table
- TEST Cluster
- TEST Cluster
- TEST Cluster
- tum io
- Imu Rotation Result
- rebuild Ahrs Preintegration
- Live Track
- Camera Parameters
- Imu Parameters
- Implemented Keyframe Strategy
- M4 4 Keyframe Shadow Cluster
- config snapshot
- Run Summary
- Run Meta
- Image Frame Event
- Fixed Lag Graph Stats
- run Offline Vo Session
- Arguments Cluster
- phad common
- Pn P to Cull Cluster
- json Cluster
- Dataset Result
- Calibration Result
- tum vi dataset
- candidate pipeline probe
- Synthetic Euroc Fixture
- synthetic stereo
- rebuild Ahrs Preintegration
- Candidate Run Result
- Keyframe Shadow Probe Impl
- M4 4 B5 Guarded Cluster
- P2b Fixed lag Candidate Design
- TEST Cluster
- Trajectory Error
- trajectory panel
- Keyframe Event
- phad euroc runner
- Mh01 Groundtruth Test
- Dataset Error
- Dataset Replay Source
- euroc mh01 rectify test
- Tiny Euroc Fixture
- Image Window
- TEST Cluster
- Sync Summary
- Trajectory Summary
- Frame Stats
- Stereo Imu Dataset Reader Impl
- Pending Decision
- TEST Cluster
- TEST Cluster
- euroc dataset
- stereo imu dataset
- Trajectory Panel
- phad vio
- collect Landmark Factors
- Zombie Track by Block Cluster
- Vector Cluster
- Window Frame
- Stereo Tracker Impl
- Track Observation
- read tum
- segment ate decomp
- Target VIO Pipeline
- M3 3 Checkpoint Lineage
- TEST Cluster
- Mh01 Frontend Test
- Dataset Reader Error
- read errors
- keyframe shadow probe
- observe Cluster
- drop Tracks as Main Cluster
- M3 3 Slice 5b Cluster
- Guarded Full inertial Design
- Code Identity
- Vio Update Result
- parse Sensor Transform
- decode Image
- Sensor Source Error
- Offline Vo Session Contract
- Arguments Cluster
- Config Snapshot
- Camera Record
- read csv
- phad vo bench cli test
- Keyframe Rotation Evidence
- M4 4 Gyro Fixed Cluster
- Persistent Fixed Lag Shadow
- Gyro Bias Evidence
- Benchmark Checkpoint Policy
- Stereo Landmark Depth Chain
- Trajectory Panel
- trajectory panel
- Impl Cluster
- GTSAM VIO Backend Decision
- Boundary Robustification Negative Result
- Sad Peak
- pixels Cluster
- stereo imu dataset builder
- Implementation Plan Conventions

## God Nodes (most connected - your core abstractions)
1. `StereoVoEstimator::Impl` - 99 edges
2. `Timestamp` - 90 edges
3. `RectifiedStereoCalibration` - 82 edges
4. `Arguments` - 57 edges
5. `UpdateDiagnostics` - 57 edges
6. `EstimatorOptions` - 55 edges
7. `TEST()` - 50 edges
8. `CandidateFixedLagEstimator::Impl` - 49 edges
9. `Trajectory` - 49 edges
10. `ShadowRow` - 43 edges

## Surprising Connections (you probably didn't know these)
- `TEST()` --calls--> `process`  [INFERRED]
  tests/apps/candidate_pipeline_test.cpp → apps/candidate_pipeline.hpp
- `TEST()` --calls--> `write`  [INFERRED]
  tests/apps/fixed_lag_shadow_probe_test.cpp → apps/fixed_lag_shadow_probe.hpp
- `applyDropout()` --references--> `StereoObservation`  [INFERRED]
  apps/offline_vo_session.cpp → phad/estimator/types.hpp
- `runOfflineVoSession()` --references--> `TimedPose`  [INFERRED]
  apps/offline_vo_session.cpp → phad/common/trajectory.hpp
- `runOfflineVoSession()` --calls--> `dropTracks`  [INFERRED]
  apps/offline_vo_session.cpp → phad/frontend/stereo_tracker.hpp

## Import Cycles
- None detected.

## Hyperedges (group relationships)
- **Target VIO Pipeline Components** — docs_architecture_sensor_adapters, docs_architecture_sensor_synchronizer, docs_architecture_visual_inertial_frontend, docs_architecture_gtsam_vio_estimator, docs_architecture_evaluation_and_visualization [EXTRACTED 1.00]
- **M3.3 Failure Debt Motivates M4** — docs_benchmark_m3_3_prem4_diag_census_cbb4505_402d1925_census_rejected, docs_benchmark_m3_3_prem4_round2_8906684_402d1925_round2_candidates_rejected, docs_benchmark_m3_3_prem4_round2_8906684_402d1925_reanchor_alignment_tax, docs_benchmark_m3_3_prem4_round2_8906684_402d1925_m4_structural_fix [INFERRED 0.85]
- **M3.3 Hardening Progression** — docs_plans_2026_08_01_m3_3_slice2_right_match_c8e74511_plan_one_dimensional_sad, docs_plans_2026_08_01_m3_3_slice3_pnp_0154cd20_plan_pnp_ransac_initialization, docs_plans_2026_08_01_m3_3_slice4_outlier_cull_840bf39c_plan_mean_reprojection_cull [EXTRACTED 1.00]
- **Landmark and Track Lifecycle Mitigations** — docs_benchmark_m3_3_slice_4f_c446ac5_a5e90dc7_skip_drop, docs_benchmark_m3_3_zombie_age_4cf55ca_773ea011_zombie_drop_age, docs_plans_2026_08_02_m3_3_slice4c_cull_track_drop_5a3a4e09_plan_block_rebirth_and_drop [INFERRED 0.85]
- **M4 Inertial Diagnosis Paths** — docs_benchmark_m4_3_readme_static_imu_initialization, docs_benchmark_m4_4_readme_gyro_only_recovery, docs_benchmark_m4_4_readme_fixed_lag_shadow [INFERRED 0.85]
- **Post-M3.3 Debt to M4 Dependency Chain** — docs_plans_2026_08_04_m3_3_pnp_stereo_arbitration_4721cb40_plan_pnp_stereo_consistency_arbitration, docs_plans_2026_08_05_m3_3_slice5_keyframe_790dd106_plan_m3_3_slice_5_keyframe_strategy, docs_plans_2026_08_05_post_m3_3_arbitration_work_plan_cb338da4_plan_m4_phase_decomposition [EXTRACTED 1.00]
- **M4 IMU Vertical Slices** — docs_plans_2026_08_08_m4_1_sync_imu_96a99568_plan_m4_1_stereo_imu_packet_sync, docs_plans_2026_08_08_m4_2_estimator_imu_7026ebf_plan_m4_2_vio_estimator_state, docs_plans_2026_08_08_m4_3_static_init_8f2c91ab_plan_m4_3_stationary_initialization [EXTRACTED 1.00]
- **M4.4 Rule 4 Successor Evidence Chain** — docs_plans_2026_08_09_m4_4_rule4_imu_rotation_d6d46568_plan_rule4_negative_result, docs_plans_2026_08_09_m4_4_keyframe_epoch_gate_b0_7f102cb_plan_m4_4_keyframe_epoch_gate, docs_plans_2026_08_09_m4_4_keyframe_shadow_probe_plan_fixed_epoch_imu_shadow_probe [EXTRACTED 1.00]
- **M4.4 Diagnostic-to-Fixed-Lag Evolution** — docs_plans_2026_08_10_m4_4_vio_state_probe_b1a4c6e2_plan_vio_state_probe, docs_plans_2026_08_10_m4_4_vio_init_truth_probe_4c81e2ad_plan_initialization_truth_audit, docs_plans_2026_08_10_m4_4_gyro_visual_fusion_b_plan_gyro_first_visual_inertial_fusion, docs_plans_2026_08_10_m4_4_gyro_fixed_lag_contract_probe_c4a6d92e_plan_gyro_fixed_lag_contract_probe, docs_plans_2026_08_10_m4_4_gyro_fixed_lag_shadow_6e8a21d4_plan_gyro_fixed_lag_shadow, docs_plans_2026_08_10_m4_4_candidate_pipeline_p2b_6e003f0a_plan_candidate_fixed_lag_p2b [INFERRED 0.85]
- **EuRoC Data Entry Contract Evolution** — docs_research_euroc_dataset_loader_design_euroc_dataset_loader_design, docs_research_euroc_stereo_manifest_asymmetry_handoff_stereo_manifest_asymmetry_handoff, docs_research_euroc_stereo_manifest_asymmetry_open_source_refs_stereo_manifest_open_source_refs, docs_research_m3_2_euroc_baseline_m3_2_euroc_baseline [INFERRED 0.95]
- **M3.3 Landmark Lifecycle Probe Family** — docs_research_m3_3_cull_id_rebirth_open_source_refs_cull_id_rebirth_refs, docs_research_m3_3_defer_drop_topk_probe_design_defer_drop_topk_probe, docs_research_m3_3_evict_skip_culled_probe_design_evict_skip_culled_probe, docs_research_m3_3_full_suite_baseline_773ea011_m3_3_full_suite_baseline [INFERRED 0.95]
- **Keyframe Policy Reference Family** — docs_research_m3_3_keyframe_open_source_refs_vins_keyframe_policy, docs_research_m3_3_keyframe_open_source_refs_orbslam3_keyframe_policy, docs_research_m3_3_keyframe_open_source_refs_kimera_keyframe_policy, docs_research_m3_3_keyframe_open_source_refs_basalt_keyframe_policy, docs_research_m3_3_keyframe_design_implemented_keyframe_strategy [EXTRACTED 1.00]
- **MH_05 Diagnostic Evidence Chain** — docs_research_m3_3_mh05_failure_diagnosis_droptracks_main_factor, docs_research_m3_3_mh05_post4f_diagnosis_chronic_self_consistent_drift, docs_research_m3_3_mh05_probe_b_diagnosis_zombie_block_chain, docs_research_m3_3_post4g_next_knife_candidates_aged_zombie_drop_candidate [INFERRED 0.95]
- **PnP Arbitration Evidence Stack** — docs_research_m3_3_mh02_divergence_diagnosis_pnp_cull_authorization_chain, docs_research_m3_3_pnp_stereo_consistency_open_source_refs_gtsam_stereo_residual_contract, docs_research_m3_3_pnp_stereo_consistency_design_pnp_stereo_arbitration, docs_research_m3_3_pnp_stereo_arbitration_results_arbitration_validation [INFERRED 0.95]
- **M3.3 Frontend and Geometry Hardening Chain** — docs_research_m3_3_slice2_right_match_design_slice2_right_match_design, docs_research_m3_3_slice3_pnp_design_slice3_pnp_design, docs_research_m3_3_slice6_frontend_hardening_design_slice6_frontend_hardening_design, docs_research_m3_3_slice7_multiframe_triangulation_design_slice7_multiframe_triangulation_design [INFERRED 0.85]
- **M3.3 Landmark Cull and Recovery Family** — docs_research_m3_3_slice4_outlier_cull_design_slice4_outlier_cull_design, docs_research_m3_3_slice4b_outlier_reopt_design_slice4b_outlier_reopt_design, docs_research_m3_3_slice4c_cull_track_drop_design_slice4c_cull_track_drop_design, docs_research_m3_3_slice4d_cull_threshold_design_slice4d_cull_threshold_design, docs_research_m3_3_slice4e_multiround_reopt_design_slice4e_multiround_reopt_design, docs_research_m3_3_slice4f_skip_drop_design_slice4f_skip_drop_design, docs_research_m3_3_slice4g_zombie_drop_design_slice4g_zombie_drop_design [EXTRACTED 1.00]
- **M3.3 Keyframe Failure to All-Frame BA** — docs_research_m3_3_slice5_full_suite_results_slice5_full_suite_results, docs_research_m3_3_slice5_keyframe_research_refs_slice5_keyframe_research_refs, docs_research_m3_3_slice5b_pose_refine_design_slice5b_pose_refine_design, docs_research_m3_3_slice5c_all_frames_ba_design_slice5c_all_frames_ba_design [INFERRED 0.95]
- **M3.3 Frontend Collapse and Recovery Evidence Chain** — docs_research_m3_3_v_series_frontend_diagnosis_frontend_tracking_diagnosis, docs_research_m3_3_v102_regression_attribution_keyframe_failure_self_sustain_chain, docs_research_m3_3_vo_collapse_diagnosis_zero_shared_absorbing_state, docs_research_m3_3_vo_hardening_design_reanchor_seed_state_machine [INFERRED 0.85]
- **M4.4 Candidate Pipeline Progression** — docs_research_m4_4_candidate_pipeline_design_atomic_candidate_pipeline, docs_research_m4_4_candidate_pipeline_p2a_result_batch_twin_equivalence, docs_research_m4_4_candidate_pipeline_p2b_design_fixed_lag_candidate_lifecycle, docs_research_m4_4_candidate_pipeline_p2b_result_fixed_lag_structure_gate_result [EXTRACTED 1.00]
- **M4.4 Full-inertial Diagnostic Chain** — docs_research_m4_4_full_inertial_fixed_bias_probe_fixed_acc_bias_negative_result, docs_research_m4_4_full_factor_cost_probe_design_imu_factor_overconfidence, docs_research_m4_4_full_inertial_consistency_covariance_probe_calibrated_covariance_negative_result [EXTRACTED 1.00]
- **M4.4 Keyframe Schedule Evidence Pipeline** — docs_research_m4_4_keyframe_schedule_probe_accepted_schedule_divergence, docs_research_m4_4_keyframe_epoch_gate_design_keyframe_epoch_transaction, docs_research_m4_4_keyframe_shadow_probe_design_frozen_epoch_shadow, docs_research_m4_4_keyframe_shadow_probe_no_schedule_guard_promotion [INFERRED 0.95]
- **M4.4 Initialization Diagnosis Chain** — docs_research_m4_4_vio_state_probe_design_pim_prediction_posterior_probe, docs_research_m4_4_vio_init_truth_probe_design_production_initialization_snapshot, docs_research_m4_4_vio_init_truth_probe_gravity_tilt_bias_coupling, docs_research_m4_4_staged_translation_alignment_probe_staged_shared_accelerometer_bias_alignment [INFERRED 0.95]
- **M4.4 Estimator History and Lifecycle Evidence** — docs_research_m4_4_velocity_history_shadow_probe_causal_velocity_history, docs_research_m4_4_gyro_fixed_lag_shadow_design_persistent_fixed_lag_shadow, docs_research_m4_4_gyro_fixed_lag_shadow_result_fixed_lag_shadow_divergence, docs_research_m4_4_vio_accuracy_degradation_diagnosis_candidate_confounders [INFERRED 0.85]
- **PHAD Stereo-VIO Sensor Pipeline** — phad_sensor_readme_source_independent_sensor_contracts, phad_io_readme_source_adapter_boundary, phad_sync_readme_stereo_imu_synchronization, phad_camera_readme_runtime_stereo_geometry, phad_frontend_readme_stereo_feature_tracking, phad_estimator_readme_gyro_visual_fixed_window_backend [EXTRACTED 1.00]
- **PHAD Evaluation and Regression Pipeline** — phad_common_readme_trajectory_timestamp_invariants, phad_eval_readme_fixed_scale_trajectory_evaluation, phad_bench_readme_regression_artifact_contract [INFERRED 0.95]
- **M4.4 Error Localization Chain** — docs_research_m4_4_vio_vo_error_decomposition_reference_common_support_state_decomposition, docs_research_m4_4_window_history_counterfactual_probe_velocity_dominant_displacement_error, phad_estimator_readme_gyro_visual_fixed_window_backend, docs_roadmap_m4_4_gyro_first_status [INFERRED 0.95]
- **Headless Visualization Layering** — phad_viz_agents_viz_agent_contract, phad_viz_readme_viz_module_contract, phad_viz_readme_trajectorypanel, phad_viz_readme_imagewindow [EXTRACTED 1.00]
- **Offline Diagnostics and Validation Toolchain** — scripts_agents_scripts_agent_contract, scripts_readme_offline_analysis_scripts, scripts_requirements_offline_plotting_dependencies, tests_agents_test_execution_contract [INFERRED 0.85]
- **M3.3 Keyframe Evaluation Loop** — scripts_readme_keyframe_probe_suite, scripts_readme_bench_table, docs_research_m3_3_slice5_dynamic_threshold_refs_dynamic_keyframe_threshold_research, docs_research_m3_3_slice5_open_benchmark_comparison_open_benchmark_comparison [INFERRED 0.85]

## Communities (211 total, 4 thin omitted)

### Community 0 - "IMU Translation Probes"
Cohesion: 0.05
Nodes (102): BiasAt, _alignment_system(), AlignmentResult, BiasAlignmentResult, build_intervals(), _correct_intervals_for_bias(), evaluate_alignment(), EvaluationResult (+94 more)

### Community 1 - "VIO State Analysis"
Cohesion: 0.07
Nodes (56): Pattern, AnalysisResult, analyze(), _boolean(), BucketResult, correction_summary(), CorrectionSummary, _evaluate_pair() (+48 more)

### Community 2 - "Keyframe Schedule Probes"
Cohesion: 0.07
Nodes (56): Counter, build_parser(), compare_traces(), _event(), _first_order_mismatch(), _first_set_departure(), _fmt_float(), _fmt_rate() (+48 more)

### Community 3 - "EuRoC Dataset Tests"
Cohesion: 0.05
Nodes (54): AdapterOpensCalibrationSummaryAndSequentialMeasurements, CopiedHandleReturnsCalibrationAndSummaryByValue, DefersCorruptPngFailureUntilSequentialRightImageConsumption, EurocAdapterTest, EurocInspectTest, ImuAndImageStreamsEndIndependentlyAndStably, IndependentDatasetsAndReadersAreDeterministic, MoveOnlyReaderOutlivesDatasetHandle (+46 more)

### Community 4 - "VIO Initialization Probes"
Cohesion: 0.13
Nodes (44): _add(), analyze(), AuditResult, _body_up(), _bracket(), _conjugate(), _dot(), _finite() (+36 more)

### Community 5 - "Stereo Calibration Geometry"
Cohesion: 0.08
Nodes (49): AcceptsValidInputs, RigidTransform, RigidTransform, RectifiedStereoCalibration, baselineM, create, cxPixels, cyPixels (+41 more)

### Community 6 - "Gyro Bias Alignment"
Cohesion: 0.13
Nodes (45): Quaternion, _add(), analyze(), analyze_rolling(), BiasEstimate, build_pairs(), estimate_bias(), _finite() (+37 more)

### Community 7 - "Benchmark CLI Configuration"
Cohesion: 0.04
Nodes (50): Arguments, allow_culled_rebirth, bench_root, candidate, config_label, defer_drop_topk, dropout_frames, dropout_keep_ratio (+42 more)

### Community 8 - "VIO State Probe Runtime"
Cohesion: 0.06
Nodes (45): FusionMode, int64_t, Isometry3d, ofstream, optional, path, Quaterniond, string_view (+37 more)

### Community 9 - "Camera Model Tests"
Cohesion: 0.05
Nodes (46): AppliesEquidistantDistortionGolden, AppliesRadialTangentialDistortionGolden, CameraModelResultTest, CameraModelTest, EquidistantBackProjectionDoesNotAcceptPixelAmplifiedResidual, ExposesValueAndStructuredErrorBranches, CameraModel, backProject (+38 more)

### Community 10 - "Stereo VO Estimator"
Cohesion: 0.04
Nodes (45): InitPhase, PreintegratedRotationParams, unique_ptr, StereoVoEstimator::Impl, ahrs_params, body_P_sensor, calibration, culled_ids_ (+37 more)

### Community 11 - "M3.3 Benchmark Evidence"
Cohesion: 0.05
Nodes (47): Skip Drop for Large Cull, M3.3 Slice ④f Benchmark, Deferred Drop Equivalence, M3.3 Slice ④g Benchmark, All Frames Sliding Window BA, M3.3 Slice ⑤c Benchmark, LK CLAHE and F-RANSAC Frontend Hardening, M3.3 Slice ⑥ Benchmark (+39 more)

### Community 12 - "M3.3 Slice Plans"
Cohesion: 0.05
Nodes (44): M3.3 Slice 4d Mean-Cull Threshold, MH_01 Threshold Gate, Bounded Cull-LM Loop, M3.3 Slice 4e Multiround Reoptimization, M3.3 Slice 4f Skip Drop, Mass-Cull Drop Gate, M3.3 Slice 4g Deferred Zombie Drop, Pending Drop Lifecycle (+36 more)

### Community 13 - "Estimator Diagnostics Types"
Cohesion: 0.05
Nodes (44): FusionMode, pair, UpdateDiagnostics, bias_acc, bias_gyro, culled_landmark_ids, fixed_lag_shadow, fusion_mode (+36 more)

### Community 14 - "IMU Factor Construction"
Cohesion: 0.05
Nodes (41): SharedNoiseModel, makeImuPosePriorNoise(), makePriorNoise(), makeStereoNoise(), EstimatorOptions, block_culled_rebirth, enable_accumulated_seed, enable_fixed_lag_shadow (+33 more)

### Community 15 - "Probe B Artifacts"
Cohesion: 0.07
Nodes (37): appendCommaIfNeeded(), path, string, uint64_t, vector, formatDouble(), formatShiftTopArray(), formatUint64Array() (+29 more)

### Community 16 - "Stereo IMU Synchronizer"
Cohesion: 0.09
Nodes (38): vector, StereoImuPacket, frame, imu_gap, samples, t_prev, CameraId, optional (+30 more)

### Community 17 - "Keyframe Selection Logic"
Cohesion: 0.08
Nodes (36): AcceptedNonKeyframeUpdatesCausalPose, AcceptedSnapshotKeepsPreviousCausalPose, AcceptsValidatedNonIdentityExtrinsicCalibration, AlternativeEvidenceRequiresPendingFrame, AlternativeIdentityMatchesProductionEvidence, Impl, unique_ptr, KeyframeEpochGate (+28 more)

### Community 18 - "Sensor Frame Contracts"
Cohesion: 0.09
Nodes (12): optional, string, variant, EurocMh01IntegrationTest, LoadsAuditedSummaryAndFirstSequentialImageFrames, LoadsManifestCalibrationAndSampleImages, map, EndOfStream (+4 more)

### Community 19 - "Architecture Contracts"
Cohesion: 0.07
Nodes (37): Combined IMU Bias Transition Contract, Common-Support State Error Decomposition, M4.4 VO/VIO Error Decomposition Reference, M4.4 Window-History Counterfactual Probe, Velocity-Dominant Inertial Displacement Error, B-Family Stereo Pairing, Stereo Pair Synchronizer Design, PHAD-VIO Implementation Roadmap (+29 more)

### Community 20 - "Synthetic IMU Tests"
Cohesion: 0.13
Nodes (35): function, alignmentSample(), alignmentStates(), constantRotationMotion(), constantVelocityMotion(), int64_t, Isometry3d, LandmarkId (+27 more)

### Community 21 - "Camera Parameter Models"
Cohesion: 0.07
Nodes (32): PinholeEquidistantParameters, cxPixels, cyPixels, fxPixels, fyPixels, k1, k2, k3 (+24 more)

### Community 22 - "Shadow Probe Records"
Cohesion: 0.06
Nodes (35): int64_t, size_t, ShadowRow, bias_gyr, common, epoch_after, epoch_before, epoch_committed (+27 more)

### Community 23 - "TUM VI Dataset Tests"
Cohesion: 0.10
Nodes (26): CorruptImageFailsLazilyAndIsSticky, OpensNormalizedCalibrationImuAndUint16Images, PreservesUnsupportedModelCodesBeforeFactories, RejectsInvalidKalibrTransformAndDistortion, cameraFromSensorId(), AcceptsUnequalLengthCameraManifests, CalibrationErrorCode, CameraId (+18 more)

### Community 24 - "M3.3 Algorithm Designs"
Cohesion: 0.07
Nodes (35): One-Dimensional SAD Stereo Matching, M3.3 Slice 2 Right-Match Hardening Design, Explicit Disparity Search Consensus, M3.3 Slice 2 Right-Match Open-Source References, M3.3 Slice 3 PnP Baseline, PnP-RANSAC Current-Frame Inlier Mask, M3.3 Slice 3 PnP and RANSAC Design, Pose Initialization and Geometry Validation Separation (+27 more)

### Community 25 - "VIO Diagnostic State"
Cohesion: 0.06
Nodes (35): GyroStateDiagnostics, graph_initial_T_W_B, imu_dt_s, imu_sample_count, imu_t_i_ns, imu_t_j_ns, predicted_T_W_B, prediction_bias_gyro (+27 more)

### Community 26 - "VIO Bias Analysis"
Cohesion: 0.16
Nodes (28): BiasBucket, BiasSample, DiagBias, _finite(), GroundtruthBias, _integer(), join_bias(), load_diag() (+20 more)

### Community 27 - "Camera Projection API"
Cohesion: 0.15
Nodes (22): CameraModelError, code, detail, CameraModelResult, m_storage, Vector2d, Vector3d, DistortedPoint (+14 more)

### Community 28 - "Candidate VIO Pipeline"
Cohesion: 0.07
Nodes (30): CandidateCreateResult, m_storage, CandidateError, code, detail, frame_index, timestamp, CandidatePipeline (+22 more)

### Community 29 - "Trajectory Data Model"
Cohesion: 0.10
Nodes (25): vector, Isometry3d, vector, TimedPose, T_W_B, timestamp, Trajectory, kRotationTolerance (+17 more)

### Community 30 - "Offline VO Session"
Cohesion: 0.07
Nodes (32): optional, string, vector, OfflineVoSessionResult, candidate, candidate_wall_s, counts, diag (+24 more)

### Community 31 - "Relative Pose Evaluation"
Cohesion: 0.07
Nodes (29): computeStats(), vector, ErrorStats, max, mean, median, rmse, stddev (+21 more)

### Community 32 - "Fixed Lag Shadow Analysis"
Cohesion: 0.15
Nodes (27): analyze(), _buckets(), _is_zero(), load(), main(), _parse_bool(), _parse_float(), _parse_int() (+19 more)

### Community 33 - "Dataset Replay Tests"
Cohesion: 0.12
Nodes (19): DatasetReplaySourceTest, EmitsAllEventsInOrderWithImuFirstAtEqualTimestamp, EmptyDatasetReturnsStableEndOfStream, ExposesCalibrationThroughSensorSourceSeam, LazilyDecodesAndRetainsTerminalReadError, ObservedEventKind, OwnsReaderAndImagePixelsAfterDatasetHandleDestruction, SensorEvent (+11 more)

### Community 34 - "Candidate Frame Diagnostics"
Cohesion: 0.07
Nodes (31): CandidateFrameDiagnostics, culled_count, disparity_count, dropped_track_count, epoch_committed, frame_index, fusion_mode, graph (+23 more)

### Community 35 - "Keyframe Epoch Gate"
Cohesion: 0.10
Nodes (24): int64_t, KeyframeRule, Matrix3d, optional, uint64_t, unordered_map, EpochScheduler, EpochState (+16 more)

### Community 36 - "Outlier Reoptimization Tests"
Cohesion: 0.13
Nodes (30): CheiralityOnlySkipsReopt, CullsAfterReoptLm, DefaultsMaxReoptsThreeAndRoundsZero, DisabledSkipsReopt, Lm2FailureFallsBackToLm1Cull, MaxOneCapsReoptRounds, MaxZeroSkipsReopt, NoCullSkipsReopt (+22 more)

### Community 37 - "Gyro Visual Tests"
Cohesion: 0.07
Nodes (30): AccelerometerDoesNotAffectGyroVisualTrajectory, ConstantRotationMatchesGroundTruth, ConstantVelocityMatchesGroundTruth, DisabledImuReproducesVisionChain, DropoutInjectionFreezesWithGyroVisual, DropoutInjectionFreezesWithoutImu, EmptyObservationsAreRejectedWithoutTranslationConstraint, FixedLagShadowIsReadOnlyBoundedAndExplicit (+22 more)

### Community 38 - "PnP Estimator Tests"
Cohesion: 0.11
Nodes (29): AcceptsPnpWithinStereoNoise, DisabledMatchesConstantVelocity, FallsBackWhenInliersBelowMin, FallsBackWhenPnpStereoRmsIsWorseThanGuess, FallsBackWhenTooFewShared, MaskedObsKeepsTrackTimesButMayDropFromGraph, PnpMasksSharedOutliers, PnpSucceedsOnCleanMotion (+21 more)

### Community 39 - "Stereo Rectifier"
Cohesion: 0.10
Nodes (26): cameraMatrix(), CameraModelErrorCode, Mat, string, unique_ptr, distCoeffs(), fromGrayMat(), Impl (+18 more)

### Community 40 - "Candidate Pipeline Tests"
Cohesion: 0.08
Nodes (28): AllowsIncompleteImuIntervalAcrossGap, CandidateOnlyPerturbationDoesNotAffectControl, CandidatePipelineTest, DISABLED_ContinuesAfterEstimatorFrameFailure, FixedLagBiasChainAccumulatesInterKeyframeImu, FixedLagBiasChainFusesGyroAndStaysBounded, FixedLagBiasChainSkipsGyroAfterGap, FixedLagEstimatorGatesOutlierObservations (+20 more)

### Community 41 - "VO Diagnostic Rows"
Cohesion: 0.07
Nodes (29): int64_t, uint32_t, VoDiagRow, bias_acc_x, bias_acc_y, bias_acc_z, bias_gyro_x, bias_gyro_y (+21 more)

### Community 42 - "Gyro Graph Cost Diagnostics"
Cohesion: 0.07
Nodes (29): GyroGraphCostDiagnostics, bias_prior_posterior_cost, bias_prior_posterior_norm, boundary_initial_cost, boundary_initial_residual_norm_rad, boundary_initial_whitened_norm, boundary_posterior_cost, boundary_posterior_residual_norm_rad (+21 more)

### Community 43 - "Reanchor Estimator Tests"
Cohesion: 0.12
Nodes (25): AccumulatedSeedingSeedsAfterSparseFrames, AnchorFollowsConstantVelocityOption, CtorRejectsMinSeedObservationsBelowOne, FirstSegmentSeedGate, Impl, unique_ptr, StereoVoEstimator, m_impl (+17 more)

### Community 44 - "Candidate Fixed Lag Estimator"
Cohesion: 0.07
Nodes (27): CandidateFixedLagEstimator::Impl, active_landmarks, ahrs_params, bias_prior_noise, body_P_sensor, frame_count, gating_threshold_px, has_root (+19 more)

### Community 45 - "Candidate Pipeline Runtime"
Cohesion: 0.09
Nodes (25): hasValue, CandidateCreateResult::operator bool() const noexcept(), finish, CandidatePipeline::Impl, counts, diagnostics, error, estimator (+17 more)

### Community 46 - "Fixed Lag Shadow Probe"
Cohesion: 0.10
Nodes (24): FixedLagShadowReset, FusionMode, int64_t, Isometry3d, ofstream, optional, path, string_view (+16 more)

### Community 47 - "Keyframe Evidence Types"
Cohesion: 0.08
Nodes (25): int64_t, KeyframeRule, KeyframeDecision, evidence, rule, selected, ticket, KeyframeEvidence (+17 more)

### Community 48 - "Absolute Trajectory Evaluation"
Cohesion: 0.09
Nodes (23): alignSe3(), centroid(), Isometry3d, vector, Vector3d, computeAte(), Matrix3d, rotationErrorDeg() (+15 more)

### Community 49 - "Synchronizer Unit Tests"
Cohesion: 0.07
Nodes (27): BoundedQueueDropsOldestOnOverflow, EqualLengthExactPairsAll, ExactTolRejectsOneNanosecondSkew, FirstPacketIsZeroSegmentAndExpiredImuDropped, FlushCountsAllRemainingNotJustFront, FlushCountsRemainingImuAsDropped, ImuDuplicateIsStickyOnImuPath, ImuGapBeyondThresholdMarksPacket (+19 more)

### Community 50 - "Outlier Cull Tests"
Cohesion: 0.13
Nodes (26): CheiralityClearsWindowObservations, CullsPersistentHighReprojLandmark, DefaultsEnableReoptAndDiagFlagsOff, DisabledSkipsMeanReprojCull, KeepsInliersUnderThreshold, PoseEqualsFirstLmWithoutReopt, RejectsNonPositiveOutlierAvgReproj, RepeatCullIncrementsTotalNotUnique (+18 more)

### Community 51 - "Stereo Tracker Tests"
Cohesion: 0.09
Nodes (26): DoesNotReuseIdsAfterTracksLeaveFov, DropTracksRemovesIdsAndIgnoresUnknown, FarPointOutsideSearchIsNoRightMatch, MarkEvictableFreesSlotsForNewDetections, MissingRightBlobBecomesNoRightMatch, RefillsTowardMaxTracksAfterMassExodus, RejectsNegativeRowTol, RejectsNegativeUniqRatio (+18 more)

### Community 52 - "Synchronizer Diagnostics Contracts"
Cohesion: 0.08
Nodes (25): deque, int64_t, size_t, uint64_t, StereoPairDiagnostics, dropped_imu, dropped_imu_overflow, dropped_left (+17 more)

### Community 53 - "Benchmark Configuration Tests"
Cohesion: 0.08
Nodes (26): CandidateFlagDoesNotChangeConfigHash, CandidateWritesIndependentArtifacts, DeferDropTopkDoesNotChangeConfigHash, EvictSkipCulledDoesNotChangeConfigHash, FixedLagShadowProbePathDoesNotChangeConfigHash, FixedLagShadowProbeRejectsNoImu, FixedLagShadowProbeRequiresPath, KeyframeShadowPathDoesNotChangeConfigHash (+18 more)

### Community 54 - "Benchmark Run Identity Tests"
Cohesion: 0.09
Nodes (21): CleanCommitSegment, CleanExistingForceOverwrites, CleanExistingRefuses, DirtyCommitSegment, DirtyExistingForceOverwrites, DirtyExistingOverwrites, EmptySequenceThrows, MissingSummaryWrites (+13 more)

### Community 55 - "Gyro Objective Analysis"
Cohesion: 0.18
Nodes (20): analyze(), _boundary_comparison(), _close(), _load_rows(), main(), _parse_float(), _parse_int(), ProbeError (+12 more)

### Community 56 - "EuRoC Inspection CLI"
Cohesion: 0.11
Nodes (22): array, optional, RigidTransform, string_view, main(), printArray(), printCameraCalibration(), printTimestamp() (+14 more)

### Community 57 - "Pose Transform Tests"
Cohesion: 0.09
Nodes (25): AteTest, ConstantTranslationDriftEqualsDriftPerDelta, ConstantYawDriftEqualsDriftPerDelta, FailsWhenNoPairSpansTheDelta, PairsTimestampsOffTheDeltaGrid, PropagatesDegenerateAlignment, RecoversKnownRigidPerturbation, RejectsNonPositiveDelta (+17 more)

### Community 58 - "Timestamp Association"
Cohesion: 0.12
Nodes (22): int64_t, associate(), Association, dropped_out_of_range, dropped_over_threshold, droppedTotal, est_total, matchRate (+14 more)

### Community 59 - "Keyframe Shadow Tests"
Cohesion: 0.13
Nodes (24): RejectsIoAndTransactionMisuse, RetainsRejectedSegmentAndUsesCausalBias, int64_t, Matrix3d, path, size_t, string, unordered_map (+16 more)

### Community 61 - "Session Frame Counters"
Cohesion: 0.08
Nodes (24): FrameCounts, deferred_drop_ids, deferred_drops, drops_skipped, evictable_marked, failed, image_frames, init_pending_frames (+16 more)

### Community 62 - "Estimator Measurement Types"
Cohesion: 0.12
Nodes (19): Isometry3d, optional, StereoPoint2, toPose3(), toStereoPoint(), LandmarkId, vector, Vector2d (+11 more)

### Community 63 - "Benchmark Run Summary"
Cohesion: 0.09
Nodes (22): toStatBlock(), optional, MetricReport, rot_deg, trans, StageTiming, max, mean (+14 more)

### Community 64 - "Estimator Diagnostics Tests"
Cohesion: 0.15
Nodes (22): BehindCameraCountedAndSequenceContinues, LowConnectivityFlagWhenSharedBelowThreshold, ObservationTimestampsAccumulateById, RecoversBodyPoseNotLeftCamera, RejectedFrameSkippedByConstantVelocity, StereoVoDiagnostics, StereoVoExtrinsics, int64_t (+14 more)

### Community 65 - "Multi Frame Triangulation"
Cohesion: 0.12
Nodes (20): BuildGraphSkipsZeroDisparity, MultiFrameTriangulationTest, int64_t, Isometry3d, LandmarkId, uint32_t, vector, Vector2d (+12 more)

### Community 66 - "Estimator Type Tests"
Cohesion: 0.10
Nodes (19): EstimatorTypes, KeyframeMeasurementConstructs, KeyframeUpdateTest, LandmarkIdAliasesCommon, NonKeyframeAllNewIdsRejected, NonKeyframeBeforeInitRejected, NonKeyframeEntersWindow, NonKeyframePreservesPoseChain (+11 more)

### Community 67 - "Fixed Lag Diagnostics"
Cohesion: 0.09
Nodes (23): FixedLagShadowDiagnostics, active, batch_window_size, bias_delta_norm, bias_fixed, bias_present, current_epoch, cutoff_epoch (+15 more)

### Community 68 - "Stereo Tracker Options"
Cohesion: 0.09
Nodes (23): StereoTrackerOptions, enable_census, enable_clahe, enable_exposure_normalize, enable_fransac, enable_median_flow, enable_zero_mean_sad, forward_backward_px (+15 more)

### Community 69 - "Dataset Internal Storage"
Cohesion: 0.11
Nodes (22): path, ImageFrameManifestEntry, image_path, timestamp, PixelType, vector, internal::StereoImuDatasetBuilder::build(), calibration (+14 more)

### Community 70 - "Offline Session Options"
Cohesion: 0.09
Nodes (22): path, uint64_t, OfflineVoSessionOptions, collect_timing, defer_drop_topk, drop_culled_tracks, dropout_frames, dropout_keep_ratio (+14 more)

### Community 71 - "Stereo Pair Stream"
Cohesion: 0.12
Nodes (20): optional, string, vector, StereoPairStream, m_flushed, m_sync, m_terminal_error, m_warned_drop (+12 more)

### Community 72 - "M2 M3 Design Research"
Cohesion: 0.11
Nodes (22): EuRoC Dataset Loader Design, Metadata Index and Lazy Image Decode, EuRoC Stereo Manifest Asymmetry Handoff, Exact Timestamp Intersection Pairing, Offline Exact Pairing Consensus, EuRoC Stereo Manifest Open-Source References, Fixed-Window Batch Bundle Adjustment, M2.3 VO Backend Design (+14 more)

### Community 73 - "M4 VIO Diagnosis Research"
Cohesion: 0.10
Nodes (22): M4.4 Gyro-Only Recovery Design, Gyro-Only Recovery, M4.4 Gyro-Visual Fusion Design, Gyro-First Stereo Bundle Adjustment, M4.4 Initialization Truth Audit Reference, Initialization Truth Contract, M4.4 Staged Translation Alignment Probe, Staged Shared Accelerometer Bias Alignment (+14 more)

### Community 74 - "Groundtruth Dataset Tests"
Cohesion: 0.15
Nodes (16): EurocGroundtruthTest, LoadsPosesAndIgnoresVelocityAndBias, RejectsHeaderOnlyCsv, RejectsMissingCsv, RejectsMissingSequenceRoot, RejectsNonFinitePosition, RejectsNonIdentityExtrinsics, RejectsUnexpectedHeader (+8 more)

### Community 75 - "TUM Trajectory IO Tests"
Cohesion: 0.10
Nodes (19): ParsesIntegerSecondsWithoutFraction, RejectsEmptyFile, RejectsNonNumericField, RejectsSubNanosecondTimestamp, RejectsWrongFieldCount, ReportsMissingFile, RoundTripPreservesNanosecondTimestamps, RoundTripPreservesPoses (+11 more)

### Community 76 - "Dataset Adapter Parsers"
Cohesion: 0.31
Nodes (21): checkIncreasing(), DatasetErrorCode, Node, optional, path, size_t, string, string_view (+13 more)

### Community 77 - "Sensor Parameter Tests"
Cohesion: 0.11
Nodes (20): AllowsPrincipalPointOutsideImageBounds, CameraParametersTest, CreatesNamedEquidistantValues, CreatesNamedRadialTangentialValues, CreatesNamedSamplingAndNoiseValues, ImuParametersTest, create, RejectsEveryNonFiniteEquidistantField (+12 more)

### Community 78 - "Candidate Artifact Writers"
Cohesion: 0.14
Nodes (19): optional, path, string, vector, csvEscape(), writeCandidateDiagCsv(), writeCandidateMeta(), CandidateArtifactsTest (+11 more)

### Community 79 - "Candidate Fixed Lag Runtime"
Cohesion: 0.17
Nodes (15): update, Isometry3d, optional, Pose3, RigidTransform, SharedNoiseModel, size_t, uint32_t (+7 more)

### Community 80 - "vio init probe"
Cohesion: 0.15
Nodes (17): ofstream, path, Quaterniond, string_view, Vector3d, Impl, unique_ptr, requireFiniteVector() (+9 more)

### Community 81 - "TEST Cluster"
Cohesion: 0.20
Nodes (20): CheiralityIdsAppearInCulledLandmarkIds, CulledIdNotRebackprojectedWhenBlocked, DefaultsBlockRebirthAndEmptyList, StereoVoCullRebirthTest, containsId(), int64_t, Isometry3d, LandmarkId (+12 more)

### Community 82 - "Ate Report"
Cohesion: 0.10
Nodes (20): AteOptions, association, rank_tolerance, AteReport, association, rot_deg, samples, T_align (+12 more)

### Community 83 - "create Cluster"
Cohesion: 0.15
Nodes (19): AcceptsStrictlyIncreasingTimestamps, optional, size_t, vector, create, firstTimestamp, lastTimestamp, size (+11 more)

### Community 84 - "phad stereo frontend probe"
Cohesion: 0.16
Nodes (19): Arguments, frames_csv, sequence_root, tracks_csv, int64_t, LandmarkId, ofstream, path (+11 more)

### Community 85 - "phad vo bench"
Cohesion: 0.22
Nodes (19): coverageRate(), path, StageTiming, string, string_view, uint64_t, flattenConfig(), imageSpanSeconds() (+11 more)

### Community 86 - "TEST Cluster"
Cohesion: 0.11
Nodes (19): CutoffIsStrictAndNewestUsesExactKey, FixedLagContractTest, IncrementalFixedLagSmoother, NewFactorIndicesTrackReusedSlots, RefreshedGlobalSeparatorSurvivesAndBecomesFixed, RemovingLastFactorLeavesTimestampOrphanAndExpiryThrows, Smoother, SnapshotRestoresSmootherAndLedgerAfterFailedCandidate (+11 more)

### Community 87 - "stereo tracker"
Cohesion: 0.25
Nodes (16): census5x5(), Mat, uint32_t, vector, hammingDist(), inBounds(), isGrayUint8(), medianLength() (+8 more)

### Community 88 - "stereo imu dataset"
Cohesion: 0.11
Nodes (17): DatasetReaderEnd, DatasetStreamSummary, count, first_timestamp, last_timestamp, optional, shared_ptr, size_t (+9 more)

### Community 89 - "fixed lag shadow probe test"
Cohesion: 0.15
Nodes (18): AllowsLandmarkFreeSegmentResetFrame, RejectsInvalidDiagnosticsWithoutAdvancingTransaction, RejectsMissingSnapshotAndOpenFailure, int64_t, path, size_t, string, uint64_t (+10 more)

### Community 90 - "Timestamp Cluster"
Cohesion: 0.12
Nodes (14): CandidateFrameInput, imu_gap, imu_samples, t_prev, span, Timestamp, m_nanoseconds, StereoFrame (+6 more)

### Community 91 - "Imu Measurement"
Cohesion: 0.14
Nodes (16): AbsoluteGyroBiasCancelsMeasurement, ConstantPositiveZRotationHasExpectedDirection, ImuRotationTest, array, ImuMeasurement, accel_mps2, gyro_radps, timestamp (+8 more)

### Community 92 - "Project Working Agreements"
Cohesion: 0.12
Nodes (18): Incremental Development, Module-Scoped Agent Guidance, Project Working Agreements, Body Frame, Camera Extrinsics, Camera Parameters, IMU Parameters, Stereo-IMU Calibration (+10 more)

### Community 93 - "phad traj eval"
Cohesion: 0.18
Nodes (17): Arguments, errors_csv_path, est_path, gt_path, gt_sequence_root, max_dt_ms, min_match_rate, rpe_delta_s (+9 more)

### Community 94 - "TEST Cluster"
Cohesion: 0.11
Nodes (18): CandidateSwitchKeepsProductionArtifactsIdentical, EmptyProbeBPathDoesNotCreateFile, FixedLagShadowProbeRejectsDisabledImuBeforeOpen, FrameCountsDefaultsIncludeSegmentFields, IllegalProbeBParentDirFailsSession, MaxFramesLimitsCountsAndCoverageSpan, MissingSequenceReturnsError, OfflineVoSessionTest (+10 more)

### Community 95 - "query Git Identity"
Cohesion: 0.17
Nodes (15): CodeIdentityTest, FILE, MissingRepoReturnsUnknownWithWarning, CommandResult, exit_code, stdout_text, path, string (+7 more)

### Community 96 - "Offline Analysis Scripts"
Cohesion: 0.13
Nodes (18): DKB-SLAM Adaptive Keyframe Threshold, M3.3 Dynamic Keyframe Threshold Research, Keyframe Threshold Phase Sensitivity, M3.3 Open Benchmark Comparison, ORB-SLAM3 Table II, Pure VO Capability Boundary, Offline Scripts Agent Contract, Venv-Only Scientific Python Stack (+10 more)

### Community 97 - "M3 3 VO Hardening Design"
Cohesion: 0.12
Nodes (18): Keyframe Failure Self-sustain Chain, V1_02 Regression Attribution, V-series Backend Robustness Review, Frontend-first Error Attribution, V-series Frontend Tracking Diagnosis, Stereo Depth Starvation, M3.3 VO Collapse Diagnosis, Zero-shared Landmark Absorbing State (+10 more)

### Community 98 - "update Fixed Lag Shadow"
Cohesion: 0.21
Nodes (6): FixedLagShadowState, Key, uint32_t, uint64_t, ShadowFactorId, ShadowFactorKind

### Community 99 - "Scene Cluster"
Cohesion: 0.16
Nodes (17): HuberReducesOutlierPosePull, PriorStaysOnOldestAndCapsWindow, RecoversTranslationAndLowersReprojRms, SingleObservationLandmarksStayOutOfGraph, Isometry3d, LandmarkId, vector, Vector3d (+9 more)

### Community 100 - "Robustness Summary"
Cohesion: 0.11
Nodes (18): RobustnessSummary, cheirality, deferred_drop_ids, deferred_drops, drops_skipped, evictable_marked, failed, low_connectivity (+10 more)

### Community 101 - "stereo vo estimator"
Cohesion: 0.26
Nodes (16): countBehindCameraLandmarks(), countCheiralityFactors(), deque, NonlinearFactorGraph, Pose3, RigidTransform, unordered_map, unordered_set (+8 more)

### Community 102 - "Image Cluster"
Cohesion: 0.13
Nodes (13): copyImage(), Mat, PixelType, uint16_t, uint8_t, variant, vector, Image (+5 more)

### Community 103 - "vio init probe test"
Cohesion: 0.16
Nodes (17): RejectsDuplicateEvent, RejectsInvalidIntervalAndPose, RejectsInvalidSnapshotWithoutAdvancingTransaction, path, ReportsOpenFailure, size_t, string, unordered_map (+9 more)

### Community 104 - "bench table"
Cohesion: 0.24
Nodes (15): _as_float(), _as_int(), build_parser(), discover_summaries(), _fmt(), _fmt_int(), load_summary(), main() (+7 more)

### Community 105 - "TEST Cluster"
Cohesion: 0.14
Nodes (16): AccessorsReturnIndependentValues, AllowsMixedModelsSizesAndDeclaredRates, CreatesValidTransformWithoutExposingRawMatrix, EnforcesHomogeneousBottomRowTolerance, EnforcesRotationAndDeterminantWithoutRepair, create, RejectsCoincidentCameraCenters, RejectsEveryNonFiniteMatrixElement (+8 more)

### Community 106 - "TEST Cluster"
Cohesion: 0.15
Nodes (16): AlignsCoplanarPointSets, AlignTest, RecoversHalfTurnRotation, RecoversIdentityForCoincidentPointSets, RecoversKnownTransform, RejectsCoincidentPointSets, RejectsCollinearPointSets, RejectsSizeMismatch (+8 more)

### Community 107 - "TEST Cluster"
Cohesion: 0.12
Nodes (16): DeferredDropCountersSerialize, DropsSkippedCounterSerialize, EvictSkipCulledCountersSerialize, FieldCompletenessWithMetrics, MissingGitCommitIsNull, NullMetricsSerializeAsNull, OutlierCullCountersSerialize, OutlierReoptCounterSerialize (+8 more)

### Community 108 - "tum io"
Cohesion: 0.29
Nodes (16): ostream, EvalErrorCode, optional, path, size_t, string, string_view, vector (+8 more)

### Community 109 - "Imu Rotation Result"
Cohesion: 0.12
Nodes (16): ImuRotationError, span, string_view, Vector3d, ImuRotationError, Matrix3d, imuRotationErrorName(), ImuRotationResult (+8 more)

### Community 110 - "rebuild Ahrs Preintegration"
Cohesion: 0.15
Nodes (11): NoiseModelFactorN<gtsam::Pose3, gtsam::Pose3,
                                          gtsam::Vector3>, OptionalMatrixType, PreintegratedAhrsMeasurements, shared_ptr, string, Vector3, Vector3d, isFinite() (+3 more)

### Community 111 - "Live Track"
Cohesion: 0.13
Nodes (16): LandmarkId, span, StereoStatus, Impl, unique_ptr, LiveTrack, disparity_px, evictable (+8 more)

### Community 112 - "Camera Parameters"
Cohesion: 0.15
Nodes (17): CameraParameters, CameraParameters::CameraParameters(), create, imageHeight, imageWidth, m_image_height, m_image_width, m_model_parameters (+9 more)

### Community 113 - "Imu Parameters"
Cohesion: 0.16
Nodes (14): ImuParameters, accNd, accRw, gyrNd, gyrRw, m_acc_nd, m_acc_rw, m_gyr_nd (+6 more)

### Community 114 - "Implemented Keyframe Strategy"
Cohesion: 0.14
Nodes (16): Continuous-Frame IMU Preintegration, Implemented Keyframe Strategy, Missing Keyframe Cooldown, Seven-Keyframe Three-Temporal Window, Basalt Keyframe Policy, Kimera-VIO Keyframe Policy, ORB-SLAM3 Keyframe Policy, VINS-Fusion Keyframe Policy (+8 more)

### Community 115 - "M4 4 Keyframe Shadow Cluster"
Cohesion: 0.16
Nodes (16): M4.4 Keyframe Epoch Gate Design, Keyframe Epoch Transaction, M4.4 Keyframe Schedule Guard Reference, Evidence Policy Admission Separation, Accepted Keyframe Schedule Divergence, M4.4 Keyframe Schedule Probe, M4.4 Keyframe Shadow Probe Design, Frozen Epoch Keyframe Shadow (+8 more)

### Community 116 - "config snapshot"
Cohesion: 0.25
Nodes (14): canonicalText, hash8, set, toJson, int64_t, string, string_view, uint64_t (+6 more)

### Community 117 - "Run Summary"
Cohesion: 0.12
Nodes (16): RunStatus, RunSummary, ate, config_hash, config_label, git_commit_short, git_dirty, robustness (+8 more)

### Community 118 - "Run Meta"
Cohesion: 0.12
Nodes (16): string, vector, RunMeta, bench_root, code, config_canonical_text, config_hash, config_json (+8 more)

### Community 119 - "Image Frame Event"
Cohesion: 0.21
Nodes (15): CameraId, ImageFrameEvent, camera, image, timestamp, CameraId, int64_t, uint8_t (+7 more)

### Community 120 - "Fixed Lag Graph Stats"
Cohesion: 0.14
Nodes (13): CandidateFixedLagEstimator, m_impl, FixedLagGraphStats, active_factor_count, active_pose_count, marginalized_pose_count, retired_landmark_count, FixedLagUpdateResult (+5 more)

### Community 121 - "run Offline Vo Session"
Cohesion: 0.21
Nodes (13): applyDropout(), optional, path, StageTiming, uint64_t, vector, percentile(), runOfflineVoSession() (+5 more)

### Community 122 - "Arguments Cluster"
Cohesion: 0.17
Nodes (14): Arguments, defer_drop_topk, diag_csv, evict_skip_culled, kf_tum_path, probe_b_path, sequence_root, tum_path (+6 more)

### Community 123 - "phad common"
Cohesion: 0.21
Nodes (15): phad_apps_tests, phad::bench, phad::camera, phad::common, phad::estimator, phad_estimator_tests, phad::eval, phad::frontend (+7 more)

### Community 124 - "Pn P to Cull Cluster"
Cohesion: 0.14
Nodes (15): Landmark 6719 Load-Bearing Transition, PnP-to-Cull Erroneous Authorization Chain, Recommended Stereo Arbitration, Post-Cull Zero-Degree Pose Debt, Frame-RMS Local-Minima Tradeoff, Known MH_03 Regression, PnP Stereo Arbitration Validation, EuRoC Arbitration Effects (+7 more)

### Community 125 - "json Cluster"
Cohesion: 0.23
Nodes (12): json, codeToJson(), RunStatus, StageTiming, string, string_view, metricToJson(), toJson (+4 more)

### Community 126 - "Dataset Result"
Cohesion: 0.22
Nodes (12): DatasetResult, m_error, m_value, optional, T, path, RigidTransform, vector (+4 more)

### Community 127 - "Calibration Result"
Cohesion: 0.15
Nodes (13): RigidTransform, invertRigidTransform(), isIdentity(), CalibrationError, code, detail, field_path, CalibrationResult (+5 more)

### Community 128 - "tum vi dataset"
Cohesion: 0.29
Nodes (14): Node, path, RigidTransform, string, string_view, imuSourceField(), open(), parseCameraCalibration() (+6 more)

### Community 129 - "candidate pipeline probe"
Cohesion: 0.35
Nodes (14): Analysis, analyze(), check_diag(), check_est(), check_meta(), load_config_window(), load_rows(), main() (+6 more)

### Community 130 - "Synthetic Euroc Fixture"
Cohesion: 0.25
Nodes (7): int64_t, path, string, SyntheticEurocFixture, kFirstTimestampNs, kStepNs, m_root

### Community 131 - "synthetic stereo"
Cohesion: 0.23
Nodes (14): uint8_t, vector, Vector2d, Vector3d, makePointGrid(), makeRectifiedCalibration(), paintGaussianBlob(), projectLeft() (+6 more)

### Community 132 - "rebuild Ahrs Preintegration"
Cohesion: 0.18
Nodes (12): backprojectWorld(), CandidatePoseAhrsFactor, m_preintegration, NoiseModelFactorN<gtsam::Pose3, gtsam::Pose3,
                                          gtsam::Vector3>, OptionalMatrixType, PreintegratedAhrsMeasurements, PreintegratedRotationParams, shared_ptr (+4 more)

### Community 133 - "Candidate Run Result"
Cohesion: 0.14
Nodes (14): CandidateCounts, failed, frames, keyframes, ok, rejected, track_only_frames, CandidateRunResult (+6 more)

### Community 134 - "Keyframe Shadow Probe Impl"
Cohesion: 0.14
Nodes (14): Matrix3d, ofstream, optional, vector, Vector3d, KeyframeShadowProbe::Impl, bias_gyr, imu_enabled (+6 more)

### Community 135 - "M4 4 B5 Guarded Cluster"
Cohesion: 0.14
Nodes (14): Effective Discrepancy Covariance, M4.4 Full-Inertial Consistency Covariance Probe, M4.4 B5 Guarded Full-Inertial, Staged Alignment Runtime Gate, M4.4 Gyro-Only Recovery, Two-Stage Visual and Gyro Estimator, Gyro-First AHRS Graph, M4.4 Gyro-First Visual-Inertial Fusion (+6 more)

### Community 136 - "P2b Fixed lag Candidate Design"
Cohesion: 0.19
Nodes (14): Candidate-owned Shadow Reference, Closed-loop Candidate Ownership, Atomic Candidate Pipeline, Candidate-owned Pipeline Design, Batch Twin Ownership Equivalence, P2a Candidate Pipeline Result, P2b Fixed-lag Candidate Design, Fixed-lag Candidate Lifecycle (+6 more)

### Community 137 - "TEST Cluster"
Cohesion: 0.23
Nodes (13): IdentityMappingForZeroDistortionPureTranslation, RejectsEquidistantCalibration, RejectsMismatchedFrameSize, StereoRectifierTest, Matrix4d, RigidTransform, makeIdentityStereo(), makeImu() (+5 more)

### Community 138 - "Trajectory Error"
Cohesion: 0.18
Nodes (11): size_t, string, T, variant, TrajectoryError, code, detail, index (+3 more)

### Community 139 - "trajectory panel"
Cohesion: 0.20
Nodes (13): Bounds, max, min, Mat, Point, size_t, Vector2d, Vector3d (+5 more)

### Community 140 - "Keyframe Event"
Cohesion: 0.15
Nodes (13): Matrix3d, optional, uint64_t, UpdateStatus, KeyframeEvent, epoch, epoch_committed, selected (+5 more)

### Community 141 - "phad euroc runner"
Cohesion: 0.24
Nodes (12): copyGrayImage(), Mat, optional, path, loadGroundtruth(), main(), PlaybackClock, m_first_timestamp (+4 more)

### Community 142 - "Mh01 Groundtruth Test"
Cohesion: 0.19
Nodes (10): ExportedGroundtruthComparesToItselfWithZeroAte, GroundtruthComparesToItselfWithZeroRpe, LoadsMonotonicGroundtruth, path, testing::Test, Mh01GroundtruthTest, m_export_path, m_root (+2 more)

### Community 143 - "Dataset Error"
Cohesion: 0.17
Nodes (12): DatasetError, cause, code, field, line_or_record_index, sensor_id, source_path, timestamp (+4 more)

### Community 144 - "Dataset Replay Source"
Cohesion: 0.29
Nodes (12): DatasetReplaySource, fail, finish, m_calibration, m_imu_lookahead, m_reader, m_terminal_state, next (+4 more)

### Community 145 - "euroc mh01 rectify test"
Cohesion: 0.22
Nodes (11): RectifiedPairsHaveAlignedRows, Mat, path, testing::Test, vector, medianAbsolute(), Mh01RectifyTest, m_root (+3 more)

### Community 146 - "Tiny Euroc Fixture"
Cohesion: 0.23
Nodes (7): int64_t, path, string, TinyEurocFixture, kFirstTimestampNs, kSecondTimestampNs, m_root

### Community 147 - "Image Window"
Cohesion: 0.26
Nodes (8): Mat, string, string, ImageWindow, isOpen, m_name, pump, show

### Community 148 - "TEST Cluster"
Cohesion: 0.17
Nodes (12): CentersATrajectoryWithoutHorizontalExtent, HoldsTheLastPoseNotAfterTheTimestamp, IgnoresTheVerticalAxis, KeepsTheSameScaleOnBothAxes, MapsWorldSpanOntoTheUsableArea, MarksThePoseAtTheRequestedTimestamp, RejectsOptionsWithoutUsableArea, RendersTheRequestedCanvas (+4 more)

### Community 149 - "Sync Summary"
Cohesion: 0.17
Nodes (12): size_t, uint64_t, SyncSummary, dropped_left, dropped_left_overflow, dropped_right, dropped_right_overflow, emitted_stereo (+4 more)

### Community 150 - "Trajectory Summary"
Cohesion: 0.17
Nodes (12): TrajectorySummary, completion_rate, coverage_rate, failed, image_frames, init_pending_frames, ok, poses_written (+4 more)

### Community 151 - "Frame Stats"
Cohesion: 0.17
Nodes (12): FrameStats, depth_rejected, detected, disparity_rejected, epipolar_median_px, epipolar_p95_px, epipolar_rejected, evicted (+4 more)

### Community 152 - "Stereo Imu Dataset Reader Impl"
Cohesion: 0.18
Nodes (10): optional, shared_ptr, StereoImuDataset::StereoImuDataset(), StereoImuDatasetReaderImpl, m_dataset, m_next_imu_index, m_next_left_index, m_next_right_index (+2 more)

### Community 153 - "Pending Decision"
Cohesion: 0.18
Nodes (11): LandmarkId, pair, size_t, vector, Vector2d, PendingDecision, observation_count, pixels (+3 more)

### Community 154 - "TEST Cluster"
Cohesion: 0.18
Nodes (11): AssociateTest, CountsSamplesBeyondThresholdInsideGroundtruthSpan, CountsSamplesOutsideGroundtruthSpan, HonoursCustomThreshold, KeepsNearestWhenGroundtruthIsDenser, MatchesIdenticalTimestamps, MatchesJitterWithinThreshold, ReportsMatchRateTooLow (+3 more)

### Community 155 - "TEST Cluster"
Cohesion: 0.18
Nodes (10): ConfigSnapshotTest, EmptyKeyThrows, EmptySnapshotHasDeterministicHash, FieldChangeChangesHash, FloatFormatsAreStable, InsertionOrderDoesNotAffectHash, MaxOutlierReoptsChangesConfigHash, SameContentSameHash (+2 more)

### Community 156 - "euroc dataset"
Cohesion: 0.31
Nodes (10): path, RigidTransform, string, euroc::open(), imuSourceField(), parseCameraCalibration(), ParsedCamera, parameters (+2 more)

### Community 157 - "stereo imu dataset"
Cohesion: 0.35
Nodes (10): CameraId, DatasetReaderResult, unique_ptr, isLeft(), reader, StereoImuDatasetReader, m_impl, peekImageTimestamp (+2 more)

### Community 158 - "Trajectory Panel"
Cohesion: 0.18
Nodes (11): Point, vector, Vector2d, TrajectoryPanel, m_background, m_origin_px, m_origin_W, m_points (+3 more)

### Community 159 - "phad vio"
Cohesion: 0.20
Nodes (10): M3.3 Learned Status, Census Fallback Rejected, Segment ATE Decomposition, M4 Structural Fix Direction, Re-Anchor Alignment Tax, Pre-M4 Round 2 Candidates Rejected, M3.3 VO Hardening, M4 IMU Integration (+2 more)

### Community 160 - "collect Landmark Factors"
Cohesion: 0.24
Nodes (7): Key, NonlinearFactorGraph, StereoPoint2, Values, toStereoPoint(), KeyTimestampMap, LandmarkCollectResult

### Community 161 - "Zombie Track by Block Cluster"
Cohesion: 0.22
Nodes (10): Chronic Self-Consistent Drift, Post-4f Diagnosis Design, Probe B Recommendation, Skip-Drop Resolved Prior Disaster, Probe B Sidecar Instrumentation, Exact Culled-Zombie Drop Recommendation, Zombie-Track by Block-Rebirth Chain, Aged-Zombie Drop Candidate (+2 more)

### Community 162 - "Vector Cluster"
Cohesion: 0.31
Nodes (6): considerShiftTopK(), LandmarkId, pair, size_t, Vector, observationTimestamps

### Community 163 - "Window Frame"
Cohesion: 0.20
Nodes (10): WindowFrame, frame_index, gyro_bias, imu_gap, imu_samples, is_keyframe, observations, t_prev (+2 more)

### Community 164 - "Stereo Tracker Impl"
Cohesion: 0.20
Nodes (9): optional, StereoTracker::Impl, calibration, has_prev, median_flow, next_id, options, prev_left (+1 more)

### Community 165 - "Track Observation"
Cohesion: 0.20
Nodes (10): LandmarkId, StereoStatus, uint32_t, Vector2d, TrackObservation, disparity_px, id, left_pixel (+2 more)

### Community 166 - "read tum"
Cohesion: 0.29
Nodes (9): main(), parse_arguments(), Namespace, ndarray, Path, 读取 TUM 文件，返回 (N, 8) 数组。, 用等边包围盒统一三个轴的比例，避免轨迹形状被拉伸。, read_tum() (+1 more)

### Community 167 - "segment ate decomp"
Cohesion: 0.38
Nodes (9): load_diag_segments(), load_est_tum(), main(), parse_args(), Path, diag.csv → {timestamp_ns: segment_id}, est.tum → [(ts_ns, line), ...],按 segment_id 分组, 返回 (ate_rmse, matched) 或 None(对齐失败/无匹配) (+1 more)

### Community 168 - "Target VIO Pipeline"
Cohesion: 0.31
Nodes (9): Controlled VIO Evolution, Evaluation and Visualization, GTSAM VIO Estimator, Inertial Initialization State Machine, Sensor Adapters, Sensor Synchronizer, Single-Thread Deterministic Execution, Target VIO Pipeline (+1 more)

### Community 169 - "M3 3 Checkpoint Lineage"
Cohesion: 0.22
Nodes (9): MH_02 Divergence Eliminated, MH_03 Known Regression, PnP Stereo Consistency Arbitration, M3.3 Checkpoint Lineage, PnP Stereo Current Estimator Default, Slice 7 Current Frontend Default, Slice 4e Partial Completion, Slice 4e Multi-Round Reoptimization (+1 more)

### Community 170 - "TEST Cluster"
Cohesion: 0.36
Nodes (8): EigenIsometryRoundTrip, GtsamSmoke, Pose3Constructs, Isometry3d, Pose3, isometryFromPose(), poseFromIsometry(), TEST()

### Community 171 - "Mh01 Frontend Test"
Cohesion: 0.31
Nodes (7): FullSequenceKeepsTracksAndIds, path, testing::Test, Mh01FrontendTest, m_root, sequenceRoot(), TEST_F()

### Community 172 - "Dataset Reader Error"
Cohesion: 0.22
Nodes (8): DatasetReaderError, cause, code, record_number, sensor_id, timestamp, DatasetReaderErrorCode, string

### Community 173 - "read errors"
Cohesion: 0.33
Nodes (8): main(), parse_arguments(), Namespace, ndarray, Path, 读取误差 CSV，返回带列名的结构化数组。, read_errors(), summarize()

### Community 174 - "keyframe shadow probe"
Cohesion: 0.43
Nodes (7): KeyframeRule, string_view, UpdateStatus, resolve, ruleName(), statusName(), writeRow()

### Community 175 - "observe Cluster"
Cohesion: 0.25
Nodes (7): span, uint64_t, Impl, unique_ptr, KeyframeShadowProbe, m_impl, observe

### Community 176 - "drop Tracks as Main Cluster"
Cohesion: 0.36
Nodes (8): Allow-Rebirth A/B Ineffective, No-Drop Plus Allow-Rebirth Collapse, No-Drop A/B Improvement, Cull Threshold Three Ineffective, Conditional-Drop Direction, Offline Comparative Diagnosis Design, dropTracks as Main MH_05 Factor, Missing Second Cull Recovery

### Community 177 - "M3 3 Slice 5b Cluster"
Cohesion: 0.36
Nodes (8): PnP-Only Non-Keyframe Design Flaw, M3.3 Slice 5 Full-Suite Results, All-Frame Pose Quality Consensus, M3.3 Slice 5 Keyframe Research References, Pose-Only Refinement and Keyframe Selection Repair, M3.3 Slice 5b Pose-Refine Design, Seven-Keyframe Three-Temporal Window, M3.3 Slice 5c All-Frames BA Design

### Community 178 - "Guarded Full inertial Design"
Cohesion: 0.36
Nodes (8): Full-inertial Factor-cost Probe, IMU Factor Overconfidence, Consistency Covariance Negative Result, Full-inertial Consistency Covariance Probe, Fixed Accelerometer Bias Negative Result, Full-inertial Fixed-bias Probe, Guarded Full-inertial Design, Staged Full-inertial Activation

### Community 179 - "Code Identity"
Cohesion: 0.25
Nodes (7): CodeIdentity, git_branch, git_commit, git_commit_short, git_dirty, optional, string

### Community 180 - "Vio Update Result"
Cohesion: 0.25
Nodes (8): optional, string, UpdateStatus, VioUpdateResult, diagnostics, estimate, message, status

### Community 181 - "parse Sensor Transform"
Cohesion: 0.36
Nodes (7): Node, path, RigidTransform, string, string_view, parseSensorTransform(), transformSourceField()

### Community 182 - "decode Image"
Cohesion: 0.39
Nodes (8): DatasetErrorCode, path, size_t, string, uint32_t, decodeImage(), makeError(), makeReaderError()

### Community 183 - "Sensor Source Error"
Cohesion: 0.25
Nodes (8): optional, string, SensorSourceError, cause, code, source_id, timestamp, SensorSourceErrorCode

### Community 184 - "Offline Vo Session Contract"
Cohesion: 0.29
Nodes (7): Apps Composition Root, diag.csv Contract, Session Keyframe Selection, M3.3 Track Lifecycle, M4.4 Shadow Probes, OfflineVoSession Contract, num_disparity Diagnostic

### Community 185 - "Arguments Cluster"
Cohesion: 0.38
Nodes (6): Arguments, output_path, sequence_root, path, main(), parseArguments()

### Community 186 - "Config Snapshot"
Cohesion: 0.29
Nodes (5): ConfigSnapshot, m_values, size_t, string, Value

### Community 187 - "Camera Record"
Cohesion: 0.29
Nodes (7): CameraRecord, image_path, line, timestamp, path, size_t, toImageManifest()

### Community 188 - "read csv"
Cohesion: 0.38
Nodes (6): main(), parse_arguments(), Namespace, ndarray, Path, read_csv()

### Community 189 - "phad vo bench cli test"
Cohesion: 0.48
Nodes (6): path, string, string_view, extractConfigHash(), readFile(), runBench()

### Community 190 - "Keyframe Rotation Evidence"
Cohesion: 0.33
Nodes (6): size_t, KeyframeRotationEvidence, common_count, compensated_parallax_px, parallax_count, raw_parallax_px

### Community 191 - "M4 4 Gyro Fixed Cluster"
Cohesion: 0.40
Nodes (6): M4.4 Candidate Fixed-Lag P2b Structural Slice, Fixed-Lag Lifecycle Health, M4.4 Gyro Fixed-Lag Contract Probe, Installed GTSAM Fixed-Lag Contract, Behavior-Isolated Fixed-Lag Shadow, M4.4 Gyro Fixed-Lag Read-Only Shadow

### Community 192 - "Persistent Fixed Lag Shadow"
Cohesion: 0.40
Nodes (6): M4.4 Gyro Fixed-Lag Shadow Design, Persistent Fixed-Lag Shadow, M4.4 Gyro Fixed-Lag Shadow Result, Fixed-Lag Shadow Divergence, Causal Velocity History, M4.4 Velocity History Shadow Probe

### Community 193 - "Gyro Bias Evidence"
Cohesion: 0.33
Nodes (6): GyroBiasEvidence, duration_s, imu_samples, R_W_B_i, R_W_B_j, Rot3

### Community 194 - "Benchmark Checkpoint Policy"
Cohesion: 0.40
Nodes (5): Documentation Workflow, Benchmark Checkpoint Schema, Benchmark Checkpoint Policy, Clean-Commit Benchmark Execution, Reproducible Config Snapshot

### Community 195 - "Stereo Landmark Depth Chain"
Cohesion: 0.50
Nodes (5): Far-Point Anchor Risk, Inverse-Depth Parameterization, Multi-Frame Triangulation, Stereo Landmark Depth Chain, V2_02 Precision Debt

### Community 196 - "Trajectory Panel"
Cohesion: 0.60
Nodes (5): Headless Rendering Test Rule, phad::viz Agent Contract, ImageWindow, TrajectoryPanel, phad::viz Module Contract

### Community 197 - "trajectory panel"
Cohesion: 0.40
Nodes (4): TrajectoryPanelOptions, height_px, margin_px, width_px

### Community 199 - "GTSAM VIO Backend Decision"
Cohesion: 0.50
Nodes (4): Batch-First Backend Evolution, Estimator Owns GTSAM Lifecycle, GTSAM VIO Backend Decision, GTSAM Stereo VIO Strategy

### Community 200 - "Boundary Robustification Negative Result"
Cohesion: 0.67
Nodes (4): Boundary Robustification Negative Result, Gyro Boundary Factor Reference, Gyro Covariance and Bias Observability, Gyro Factor Covariance Reference

### Community 201 - "Sad Peak"
Cohesion: 0.50
Nodes (4): SadPeak, ok, sad, u

### Community 202 - "pixels Cluster"
Cohesion: 0.50
Nodes (4): optional, span, pixels(), Pixel

## Knowledge Gaps
- **1107 isolated node(s):** `m_preintegration`, `options`, `smoother`, `K`, `body_P_sensor` (+1102 more)
  These have ≤1 connection - possible missing edges or undocumented components.
- **4 thin communities (<3 nodes) omitted from report** — run `graphify query` to explore isolated nodes.

## Suggested Questions
_Questions this graph is uniquely positioned to answer:_

- **Why does `Timestamp` connect `Timestamp Cluster` to `synthetic stereo`, `EuRoC Dataset Tests`, `VIO State Probe Runtime`, `Trajectory Error`, `Stereo VO Estimator`, `trajectory panel`, `phad euroc runner`, `Dataset Error`, `Stereo IMU Synchronizer`, `Keyframe Selection Logic`, `Sensor Frame Contracts`, `Image Window`, `TEST Cluster`, `Pending Decision`, `VIO Diagnostic State`, `Candidate VIO Pipeline`, `Trajectory Data Model`, `Offline VO Session`, `stereo imu dataset`, `Trajectory Panel`, `Candidate Frame Diagnostics`, `Keyframe Epoch Gate`, `Vector Cluster`, `Window Frame`, `PnP Estimator Tests`, `Reanchor Estimator Tests`, `Candidate Fixed Lag Estimator`, `Candidate Pipeline Runtime`, `Dataset Reader Error`, `Keyframe Evidence Types`, `Stereo Tracker Tests`, `Synchronizer Diagnostics Contracts`, `decode Image`, `Sensor Source Error`, `EuRoC Inspection CLI`, `Timestamp Association`, `Camera Record`, `Keyframe Shadow Tests`, `Estimator Measurement Types`, `Multi Frame Triangulation`, `Estimator Type Tests`, `Dataset Internal Storage`, `trajectory panel`, `Dataset Adapter Parsers`, `Ate Report`, `create Cluster`, `stereo tracker`, `stereo imu dataset`, `fixed lag shadow probe test`, `Imu Measurement`, `tum io`, `Imu Rotation Result`, `Image Frame Event`?**
  _High betweenness centrality (0.116) - this node is a cross-community bridge._
- **Why does `StereoVoEstimator::Impl` connect `Stereo VO Estimator` to `Gyro Bias Evidence`, `update Fixed Lag Shadow`, `Vector Cluster`, `Window Frame`, `Stereo Calibration Geometry`, `stereo vo estimator`, `rebuild Ahrs Preintegration`, `IMU Factor Construction`, `config snapshot`, `VIO Diagnostic State`, `Timestamp Cluster`, `Imu Measurement`, `Estimator Measurement Types`?**
  _High betweenness centrality (0.036) - this node is a cross-community bridge._
- **Why does `Trajectory` connect `Trajectory Data Model` to `Candidate Run Result`, `Trajectory Error`, `trajectory panel`, `phad euroc runner`, `Sensor Frame Contracts`, `TEST Cluster`, `Candidate VIO Pipeline`, `Offline VO Session`, `Relative Pose Evaluation`, `Absolute Trajectory Evaluation`, `Pose Transform Tests`, `Timestamp Association`, `trajectory panel`, `TUM Trajectory IO Tests`, `Ate Report`, `create Cluster`, `phad traj eval`, `TEST Cluster`, `TEST Cluster`, `tum io`, `Dataset Result`?**
  _High betweenness centrality (0.031) - this node is a cross-community bridge._
- **Are the 2 inferred relationships involving `Timestamp` (e.g. with `pushImage` and `TEST()`) actually correct?**
  _`Timestamp` has 2 INFERRED edges - model-reasoned connections that need verification._
- **What connects `m_preintegration`, `options`, `smoother` to the rest of the system?**
  _1107 weakly-connected nodes found - possible documentation gaps or missing edges._
- **Should `IMU Translation Probes` be split into smaller, more focused modules?**
  _Cohesion score 0.05385674931129476 - nodes in this community are weakly interconnected._
- **Should `VIO State Analysis` be split into smaller, more focused modules?**
  _Cohesion score 0.07341772151898734 - nodes in this community are weakly interconnected._