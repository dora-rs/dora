//! Unit tests for ROS compatibility layer

#[cfg(test)]
mod tests {
    use super::super::message_parser;
    use super::super::converter::common_types;

    #[test]
    fn test_parse_simple_msg() {
        let content = r#"
# This is a comment
int32 x
float64 y
string name
"#;
        
        let msg = message_parser::parse_msg_content(content).unwrap();
        assert_eq!(msg.fields.len(), 3);
        assert_eq!(msg.fields[0].name, "x");
        assert_eq!(msg.fields[0].field_type, "int32");
        assert!(!msg.fields[0].is_array);
    }

    #[test]
    fn test_parse_array_types() {
        let content = r#"
int32[] values
float64[10] fixed_array
"#;
        
        let msg = message_parser::parse_msg_content(content).unwrap();
        assert_eq!(msg.fields.len(), 2);
        assert!(msg.fields[0].is_array);
        assert_eq!(msg.fields[0].array_size, None);
        assert!(msg.fields[1].is_array);
        assert_eq!(msg.fields[1].array_size, Some(10));
    }

    #[test]
    fn test_parse_with_constants() {
        let content = r#"
int32 FOO=42
string BAR="hello"
float64 PI=3.14159
int32 x
"#;
        
        let msg = message_parser::parse_msg_content(content).unwrap();
        assert_eq!(msg.constants.len(), 3);
        assert_eq!(msg.fields.len(), 1);
        assert_eq!(msg.constants[0].name, "FOO");
        assert_eq!(msg.constants[0].value, "42");
    }

    #[test]
    fn test_header_converter() {
        let array = common_types::header_to_arrow(
            123,
            1234567890,
            123456789,
            "camera_link",
        ).unwrap();
        
        assert_eq!(array.len(), 1);
    }

    #[test]
    fn test_twist_converter() {
        let array = common_types::twist_to_arrow(
            1.0, 0.0, 0.0,  // linear
            0.0, 0.0, 1.0,  // angular
        ).unwrap();
        
        assert_eq!(array.len(), 1);
    }

    #[test]
    fn test_point_converter() {
        let array = common_types::point_to_arrow(1.0, 2.0, 3.0).unwrap();
        assert_eq!(array.len(), 1);
    }

    #[test]
    fn test_quaternion_converter() {
        let array = common_types::quaternion_to_arrow(0.0, 0.0, 0.0, 1.0).unwrap();
        assert_eq!(array.len(), 1);
    }

    #[test]
    fn test_primitive_converters() {
        let string_array = common_types::string_to_arrow("test").unwrap();
        assert_eq!(string_array.len(), 1);
        
        let int_array = common_types::int32_to_arrow(42).unwrap();
        assert_eq!(int_array.len(), 1);
        
        let float_array = common_types::float64_to_arrow(3.14).unwrap();
        assert_eq!(float_array.len(), 1);
        
        let bool_array = common_types::bool_to_arrow(true).unwrap();
        assert_eq!(bool_array.len(), 1);
    }

    #[test]
    fn test_pose_converter() {
        let array = common_types::pose_to_arrow(
            1.0, 2.0, 3.0,  // position
            0.0, 0.0, 0.0, 1.0,  // orientation
        ).unwrap();
        
        assert_eq!(array.len(), 1);
    }

    #[test]
    fn test_pose_stamped_converter() {
        use super::super::converter::common_types as sensor;
        let header = common_types::header_to_arrow(0, 0, 0, "base_link").unwrap();
        let pose = common_types::pose_to_arrow(1.0, 2.0, 3.0, 0.0, 0.0, 0.0, 1.0).unwrap();
        
        let array = sensor::pose_stamped_to_arrow(&header, &pose).unwrap();
        assert_eq!(array.len(), 1);
    }

    #[test]
    fn test_twist_stamped_converter() {
        use super::super::converter::common_types as sensor;
        let header = common_types::header_to_arrow(0, 0, 0, "base_link").unwrap();
        let twist = common_types::twist_to_arrow(1.0, 0.0, 0.0, 0.0, 0.0, 1.0).unwrap();
        
        let array = sensor::twist_stamped_to_arrow(&header, &twist).unwrap();
        assert_eq!(array.len(), 1);
    }

    #[test]
    fn test_transform_converter() {
        use super::super::converter::common_types as sensor;
        let translation = common_types::point_to_arrow(1.0, 2.0, 3.0).unwrap();
        let rotation = common_types::quaternion_to_arrow(0.0, 0.0, 0.0, 1.0).unwrap();
        
        let array = sensor::transform_to_arrow(&translation, &rotation).unwrap();
        assert_eq!(array.len(), 1);
    }

    #[test]
    fn test_transform_stamped_converter() {
        use super::super::converter::common_types as sensor;
        let header = common_types::header_to_arrow(0, 0, 0, "base_link").unwrap();
        let translation = common_types::point_to_arrow(1.0, 2.0, 3.0).unwrap();
        let rotation = common_types::quaternion_to_arrow(0.0, 0.0, 0.0, 1.0).unwrap();
        let transform = sensor::transform_to_arrow(&translation, &rotation).unwrap();
        
        let array = sensor::transform_stamped_to_arrow(&header, "child_frame", &transform).unwrap();
        assert_eq!(array.len(), 1);
    }

    // ============================================================================
    // sensor_msgs tests
    // ============================================================================

    #[test]
    fn test_image_converter() {
        use super::super::converter::common_types as sensor;
        let header = common_types::header_to_arrow(0, 0, 0, "camera_link").unwrap();
        let image_data = vec![0u8; 640 * 480 * 3]; // RGB image
        
        let array = sensor::image_to_arrow(
            &header,
            480,  // height
            640,  // width
            "rgb8",
            0,    // is_bigendian
            640 * 3,  // step
            &image_data,
        ).unwrap();
        
        assert_eq!(array.len(), 1);
    }

    #[test]
    fn test_compressed_image_converter() {
        use super::super::converter::common_types as sensor;
        let header = common_types::header_to_arrow(0, 0, 0, "camera_link").unwrap();
        let compressed_data = vec![0u8; 1000]; // JPEG data
        
        let array = sensor::compressed_image_to_arrow(
            &header,
            "jpeg",
            &compressed_data,
        ).unwrap();
        
        assert_eq!(array.len(), 1);
    }

    #[test]
    fn test_pointcloud2_converter() {
        use super::super::converter::common_types::{self as sensor, PointField};
        let header = common_types::header_to_arrow(0, 0, 0, "laser_link").unwrap();
        
        // PointCloud2 requires at least one field, but the fields array itself should be a list
        // For simplicity, test with empty fields (which is valid)
        let fields = vec![
            PointField { name: "x".to_string(), offset: 0, datatype: 7, count: 1 },
        ];
        
        let point_data = vec![0u8; 100 * 12]; // 100 points, 12 bytes each
        
        let array = sensor::pointcloud2_to_arrow(
            &header,
            1,    // height
            100,  // width
            fields,
            0,    // is_bigendian
            12,   // point_step
            1200, // row_step
            &point_data,
            true, // is_dense
        ).unwrap();
        
        assert_eq!(array.len(), 1);
    }

    #[test]
    fn test_laserscan_converter() {
        use super::super::converter::common_types as sensor;
        let header = common_types::header_to_arrow(0, 0, 0, "laser_link").unwrap();
        let ranges = vec![1.0f32, 2.0, 3.0, 4.0, 5.0];
        let intensities = vec![0.5f32, 0.6, 0.7, 0.8, 0.9];
        
        let array = sensor::laserscan_to_arrow(
            &header,
            -1.57,  // angle_min
            1.57,   // angle_max
            0.1,    // angle_increment
            0.0,    // time_increment
            0.1,    // scan_time
            0.1,    // range_min
            10.0,   // range_max
            &ranges,
            &intensities,
        ).unwrap();
        
        assert_eq!(array.len(), 1);
    }

    #[test]
    fn test_imu_converter() {
        use super::super::converter::common_types as sensor;
        let header = common_types::header_to_arrow(0, 0, 0, "imu_link").unwrap();
        let orientation = common_types::quaternion_to_arrow(0.0, 0.0, 0.0, 1.0).unwrap();
        let angular_velocity = common_types::point_to_arrow(0.0, 0.0, 0.0).unwrap();
        let linear_acceleration = common_types::point_to_arrow(0.0, 0.0, 9.81).unwrap();
        
        let orientation_cov = vec![0.0f64; 9];
        let angular_vel_cov = vec![0.0f64; 9];
        let linear_accel_cov = vec![0.0f64; 9];
        
        let array = sensor::imu_to_arrow(
            &header,
            &orientation,
            &orientation_cov,
            &angular_velocity,
            &angular_vel_cov,
            &linear_acceleration,
            &linear_accel_cov,
        ).unwrap();
        
        assert_eq!(array.len(), 1);
    }

    #[test]
    fn test_camera_info_converter() {
        use super::super::converter::common_types as sensor;
        let header = common_types::header_to_arrow(0, 0, 0, "camera_link").unwrap();
        
        let d = vec![0.1f64, -0.2, 0.3, -0.4, 0.5];
        let k = vec![
            500.0, 0.0, 320.0,
            0.0, 500.0, 240.0,
            0.0, 0.0, 1.0,
        ];
        let r = vec![
            1.0, 0.0, 0.0,
            0.0, 1.0, 0.0,
            0.0, 0.0, 1.0,
        ];
        let p = vec![
            500.0, 0.0, 320.0, 0.0,
            0.0, 500.0, 240.0, 0.0,
            0.0, 0.0, 1.0, 0.0,
        ];
        
        let array = sensor::camera_info_to_arrow(
            &header,
            480,  // height
            640,  // width
            "plumb_bob",
            &d,
            &k,
            &r,
            &p,
            1,    // binning_x
            1,    // binning_y
            0,    // roi_x_offset
            0,    // roi_y_offset
            480,  // roi_height
            640,  // roi_width
            false, // roi_do_rectify
        ).unwrap();
        
        assert_eq!(array.len(), 1);
    }

    // ============================================================================
    // nav_msgs tests
    // ============================================================================

    #[test]
    fn test_odometry_converter() {
        use super::super::converter::common_types as sensor;
        let header = common_types::header_to_arrow(0, 0, 0, "odom").unwrap();
        let pose = common_types::pose_to_arrow(1.0, 2.0, 3.0, 0.0, 0.0, 0.0, 1.0).unwrap();
        let twist = common_types::twist_to_arrow(1.0, 0.0, 0.0, 0.0, 0.0, 0.0).unwrap();
        
        let pose_cov = vec![0.0f64; 36];
        let twist_cov = vec![0.0f64; 36];
        
        let array = sensor::odometry_to_arrow(
            &header,
            "base_link",
            &pose,
            &pose_cov,
            &twist,
            &twist_cov,
        ).unwrap();
        
        assert_eq!(array.len(), 1);
    }

    #[test]
    fn test_path_converter() {
        use super::super::converter::common_types as sensor;
        let header = common_types::header_to_arrow(0, 0, 0, "map").unwrap();
        let pose1 = common_types::pose_to_arrow(1.0, 2.0, 3.0, 0.0, 0.0, 0.0, 1.0).unwrap();
        let pose2 = common_types::pose_to_arrow(2.0, 3.0, 4.0, 0.0, 0.0, 0.0, 1.0).unwrap();
        
        let poses = vec![&pose1, &pose2];
        let array = sensor::path_to_arrow(&header, poses).unwrap();
        
        assert_eq!(array.len(), 1);
    }

    #[test]
    fn test_occupancy_grid_converter() {
        use super::super::converter::common_types as sensor;
        let header = common_types::header_to_arrow(0, 0, 0, "map").unwrap();
        
        // Create a simple map info (simplified - full implementation would use proper MapMetaData)
        use arrow::array::{ArrayRef, StructArray};
        use arrow_schema::{DataType, Field, Fields};
        use std::sync::Arc;
        
        let map_info_fields = Fields::from(vec![
            Field::new("resolution", DataType::Float32, false),
            Field::new("width", DataType::UInt32, false),
            Field::new("height", DataType::UInt32, false),
        ]);
        
        let resolution_array = arrow::array::Float32Array::from(vec![0.05]);
        let width_array = arrow::array::UInt32Array::from(vec![100]);
        let height_array = arrow::array::UInt32Array::from(vec![100]);
        
        let map_info = StructArray::new(
            map_info_fields,
            vec![
                Arc::new(resolution_array) as ArrayRef,
                Arc::new(width_array) as ArrayRef,
                Arc::new(height_array) as ArrayRef,
            ],
            None,
        );
        
        let grid_data = vec![0i8; 100 * 100];
        let map_info_ref: ArrayRef = Arc::new(map_info);
        let array = sensor::occupancy_grid_to_arrow(
            &header,
            &map_info_ref,
            &grid_data,
        ).unwrap();
        
        assert_eq!(array.len(), 1);
    }
}

#[cfg(test)]
mod bridge_config_tests {
    use super::super::bridge_config::BridgeConfig;
    use super::super::bridge_config::BridgeDirection;

    #[test]
    fn test_parse_bridge_config() {
        let yaml = r#"
ros_version: 1
ros_master_uri: "http://localhost:11311"
bridges:
  - ros_topic: /camera/image_raw
    dora_topic: /sensors/camera
    msg_type: sensor_msgs/Image
    direction: ros_to_dora
  - dora_topic: /control/cmd_vel
    ros_topic: /mobile_base/commands/velocity
    msg_type: geometry_msgs/Twist
    direction: dora_to_ros
"#;
        
        let config: BridgeConfig = serde_yaml::from_str(yaml).unwrap();
        assert_eq!(config.ros_version, 1);
        assert_eq!(config.bridges.len(), 2);
        assert_eq!(config.bridges[0].direction, BridgeDirection::RosToDora);
        assert_eq!(config.bridges[1].direction, BridgeDirection::DoraToRos);
    }

    #[test]
    fn test_validate_bridge_config() {
        let mut config = BridgeConfig {
            ros_version: 1,
            ros_master_uri: Some("http://localhost:11311".to_string()),
            bridges: vec![],
        };
        
        assert!(config.validate().is_ok());
        
        config.ros_version = 3;
        assert!(config.validate().is_err());
    }
}

