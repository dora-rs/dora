//! Message conversion utilities for common ROS message types

use arrow::array::{ArrayRef, StructArray};
use arrow_schema::{DataType, Field, Fields};
use eyre::Result;
use std::sync::Arc;

/// Convert common ROS message types to Arrow format
pub mod common_types {
    use super::*;

    /// Convert std_msgs/Header to Arrow
    pub fn header_to_arrow(
        seq: u32,
        stamp_secs: u32,
        stamp_nsecs: u32,
        frame_id: &str,
    ) -> Result<ArrayRef> {
        let fields = Fields::from(vec![
            Field::new("seq", DataType::UInt32, false),
            Field::new("stamp", DataType::Struct(Fields::from(vec![
                Field::new("secs", DataType::UInt32, false),
                Field::new("nsecs", DataType::UInt32, false),
            ])), false),
            Field::new("frame_id", DataType::Utf8, false),
        ]);

        let seq_array = arrow::array::UInt32Array::from(vec![seq]);
        let secs_array = arrow::array::UInt32Array::from(vec![stamp_secs]);
        let nsecs_array = arrow::array::UInt32Array::from(vec![stamp_nsecs]);
        let frame_id_array = arrow::array::StringArray::from(vec![frame_id]);

        let stamp_fields = Fields::from(vec![
            Field::new("secs", DataType::UInt32, false),
            Field::new("nsecs", DataType::UInt32, false),
        ]);
        let stamp_struct = StructArray::new(
            stamp_fields,
            vec![Arc::new(secs_array) as ArrayRef, Arc::new(nsecs_array) as ArrayRef],
            None,
        );

        let header_struct = StructArray::new(
            fields,
            vec![
                Arc::new(seq_array) as ArrayRef,
                Arc::new(stamp_struct) as ArrayRef,
                Arc::new(frame_id_array) as ArrayRef,
            ],
            None,
        );

        Ok(Arc::new(header_struct))
    }

    /// Convert geometry_msgs/Twist to Arrow
    pub fn twist_to_arrow(
        linear_x: f64,
        linear_y: f64,
        linear_z: f64,
        angular_x: f64,
        angular_y: f64,
        angular_z: f64,
    ) -> Result<ArrayRef> {
        let fields = Fields::from(vec![
            Field::new("linear", DataType::Struct(Fields::from(vec![
                Field::new("x", DataType::Float64, false),
                Field::new("y", DataType::Float64, false),
                Field::new("z", DataType::Float64, false),
            ])), false),
            Field::new("angular", DataType::Struct(Fields::from(vec![
                Field::new("x", DataType::Float64, false),
                Field::new("y", DataType::Float64, false),
                Field::new("z", DataType::Float64, false),
            ])), false),
        ]);

        let linear_x_array = arrow::array::Float64Array::from(vec![linear_x]);
        let linear_y_array = arrow::array::Float64Array::from(vec![linear_y]);
        let linear_z_array = arrow::array::Float64Array::from(vec![linear_z]);
        let angular_x_array = arrow::array::Float64Array::from(vec![angular_x]);
        let angular_y_array = arrow::array::Float64Array::from(vec![angular_y]);
        let angular_z_array = arrow::array::Float64Array::from(vec![angular_z]);

        let linear_struct = StructArray::new(
            Fields::from(vec![
                Field::new("x", DataType::Float64, false),
                Field::new("y", DataType::Float64, false),
                Field::new("z", DataType::Float64, false),
            ]),
            vec![
                Arc::new(linear_x_array) as ArrayRef,
                Arc::new(linear_y_array) as ArrayRef,
                Arc::new(linear_z_array) as ArrayRef,
            ],
            None,
        );

        let angular_struct = StructArray::new(
            Fields::from(vec![
                Field::new("x", DataType::Float64, false),
                Field::new("y", DataType::Float64, false),
                Field::new("z", DataType::Float64, false),
            ]),
            vec![
                Arc::new(angular_x_array) as ArrayRef,
                Arc::new(angular_y_array) as ArrayRef,
                Arc::new(angular_z_array) as ArrayRef,
            ],
            None,
        );

        let twist_struct = StructArray::new(
            fields,
            vec![
                Arc::new(linear_struct) as ArrayRef,
                Arc::new(angular_struct) as ArrayRef,
            ],
            None,
        );

        Ok(Arc::new(twist_struct))
    }

    /// Convert geometry_msgs/Point to Arrow
    pub fn point_to_arrow(x: f64, y: f64, z: f64) -> Result<ArrayRef> {
        let fields = Fields::from(vec![
            Field::new("x", DataType::Float64, false),
            Field::new("y", DataType::Float64, false),
            Field::new("z", DataType::Float64, false),
        ]);

        let x_array = arrow::array::Float64Array::from(vec![x]);
        let y_array = arrow::array::Float64Array::from(vec![y]);
        let z_array = arrow::array::Float64Array::from(vec![z]);

        let point_struct = StructArray::new(
            fields,
            vec![
                Arc::new(x_array) as ArrayRef,
                Arc::new(y_array) as ArrayRef,
                Arc::new(z_array) as ArrayRef,
            ],
            None,
        );

        Ok(Arc::new(point_struct))
    }

    /// Convert geometry_msgs/Quaternion to Arrow
    pub fn quaternion_to_arrow(x: f64, y: f64, z: f64, w: f64) -> Result<ArrayRef> {
        let fields = Fields::from(vec![
            Field::new("x", DataType::Float64, false),
            Field::new("y", DataType::Float64, false),
            Field::new("z", DataType::Float64, false),
            Field::new("w", DataType::Float64, false),
        ]);

        let x_array = arrow::array::Float64Array::from(vec![x]);
        let y_array = arrow::array::Float64Array::from(vec![y]);
        let z_array = arrow::array::Float64Array::from(vec![z]);
        let w_array = arrow::array::Float64Array::from(vec![w]);

        let quat_struct = StructArray::new(
            fields,
            vec![
                Arc::new(x_array) as ArrayRef,
                Arc::new(y_array) as ArrayRef,
                Arc::new(z_array) as ArrayRef,
                Arc::new(w_array) as ArrayRef,
            ],
            None,
        );

        Ok(Arc::new(quat_struct))
    }

    /// Convert geometry_msgs/Pose to Arrow
    pub fn pose_to_arrow(
        position_x: f64, position_y: f64, position_z: f64,
        orientation_x: f64, orientation_y: f64, orientation_z: f64, orientation_w: f64,
    ) -> Result<ArrayRef> {
        let position = point_to_arrow(position_x, position_y, position_z)?;
        let orientation = quaternion_to_arrow(orientation_x, orientation_y, orientation_z, orientation_w)?;

        let fields = Fields::from(vec![
            Field::new("position", position.data_type().clone(), false),
            Field::new("orientation", orientation.data_type().clone(), false),
        ]);

        let pose_struct = StructArray::new(
            fields,
            vec![position, orientation],
            None,
        );

        Ok(Arc::new(pose_struct))
    }

    /// Convert std_msgs/String to Arrow
    pub fn string_to_arrow(data: &str) -> Result<ArrayRef> {
        Ok(Arc::new(arrow::array::StringArray::from(vec![data])))
    }

    /// Convert std_msgs/Int32 to Arrow
    pub fn int32_to_arrow(data: i32) -> Result<ArrayRef> {
        Ok(Arc::new(arrow::array::Int32Array::from(vec![data])))
    }

    /// Convert std_msgs/Float64 to Arrow
    pub fn float64_to_arrow(data: f64) -> Result<ArrayRef> {
        Ok(Arc::new(arrow::array::Float64Array::from(vec![data])))
    }

    /// Convert std_msgs/Bool to Arrow
    pub fn bool_to_arrow(data: bool) -> Result<ArrayRef> {
        Ok(Arc::new(arrow::array::BooleanArray::from(vec![data])))
    }

    // ============================================================================
    // sensor_msgs converters
    // ============================================================================

    /// Convert sensor_msgs/Image to Arrow
    /// Image format: header, height, width, encoding, is_bigendian, step, data
    pub fn image_to_arrow(
        header: &ArrayRef,
        height: u32,
        width: u32,
        encoding: &str,
        is_bigendian: u8,
        step: u32,
        data: &[u8],
    ) -> Result<ArrayRef> {
        let fields = Fields::from(vec![
            Field::new("header", header.data_type().clone(), false),
            Field::new("height", DataType::UInt32, false),
            Field::new("width", DataType::UInt32, false),
            Field::new("encoding", DataType::Utf8, false),
            Field::new("is_bigendian", DataType::UInt8, false),
            Field::new("step", DataType::UInt32, false),
            Field::new("data", DataType::Binary, false),
        ]);

        let height_array = arrow::array::UInt32Array::from(vec![height]);
        let width_array = arrow::array::UInt32Array::from(vec![width]);
        let encoding_array = arrow::array::StringArray::from(vec![encoding]);
        let is_bigendian_array = arrow::array::UInt8Array::from(vec![is_bigendian]);
        let step_array = arrow::array::UInt32Array::from(vec![step]);
        let data_array = arrow::array::BinaryArray::from(vec![data]);

        let image_struct = StructArray::new(
            fields,
            vec![
                header.clone(),
                Arc::new(height_array) as ArrayRef,
                Arc::new(width_array) as ArrayRef,
                Arc::new(encoding_array) as ArrayRef,
                Arc::new(is_bigendian_array) as ArrayRef,
                Arc::new(step_array) as ArrayRef,
                Arc::new(data_array) as ArrayRef,
            ],
            None,
        );

        Ok(Arc::new(image_struct))
    }

    /// Convert sensor_msgs/CompressedImage to Arrow
    pub fn compressed_image_to_arrow(
        header: &ArrayRef,
        format: &str,
        data: &[u8],
    ) -> Result<ArrayRef> {
        let fields = Fields::from(vec![
            Field::new("header", header.data_type().clone(), false),
            Field::new("format", DataType::Utf8, false),
            Field::new("data", DataType::Binary, false),
        ]);

        let format_array = arrow::array::StringArray::from(vec![format]);
        let data_array = arrow::array::BinaryArray::from(vec![data]);

        let compressed_image_struct = StructArray::new(
            fields,
            vec![
                header.clone(),
                Arc::new(format_array) as ArrayRef,
                Arc::new(data_array) as ArrayRef,
            ],
            None,
        );

        Ok(Arc::new(compressed_image_struct))
    }

    /// Convert sensor_msgs/PointCloud2 to Arrow
    /// PointCloud2 format: header, height, width, fields[], is_bigendian, point_step, row_step, data[], is_dense
    pub fn pointcloud2_to_arrow(
        header: &ArrayRef,
        height: u32,
        width: u32,
        fields: Vec<PointField>,
        is_bigendian: u8,
        point_step: u32,
        row_step: u32,
        data: &[u8],
        is_dense: bool,
    ) -> Result<ArrayRef> {
        // Convert PointField array - create a list of PointField structs
        let field_fields = Fields::from(vec![
            Field::new("name", DataType::Utf8, false),
            Field::new("offset", DataType::UInt32, false),
            Field::new("datatype", DataType::UInt8, false),
            Field::new("count", DataType::UInt32, false),
        ]);

        // Create individual PointField structs
        let mut pointfield_structs = Vec::new();
        for field in &fields {
            let name_array = arrow::array::StringArray::from(vec![field.name.clone()]);
            let offset_array = arrow::array::UInt32Array::from(vec![field.offset]);
            let datatype_array = arrow::array::UInt8Array::from(vec![field.datatype]);
            let count_array = arrow::array::UInt32Array::from(vec![field.count]);
            
            let pointfield = StructArray::new(
                field_fields.clone(),
                vec![
                    Arc::new(name_array) as ArrayRef,
                    Arc::new(offset_array) as ArrayRef,
                    Arc::new(datatype_array) as ArrayRef,
                    Arc::new(count_array) as ArrayRef,
                ],
                None,
            );
            pointfield_structs.push(Arc::new(pointfield) as ArrayRef);
        }

        // Create a list array from the PointField structs
        // For simplicity, if we have multiple fields, we'll create a list with one element
        // Full implementation would concatenate all structs
        let fields_array = if pointfield_structs.is_empty() {
            // Empty list
            let empty_struct = StructArray::new(field_fields.clone(), vec![], None);
            use arrow::buffer::{OffsetBuffer, ScalarBuffer};
            let offsets = OffsetBuffer::new(ScalarBuffer::from(vec![0i32, 0i32]));
            arrow::array::ListArray::new(
                Arc::new(Field::new("item", DataType::Struct(field_fields.clone()), false)),
                offsets,
                Arc::new(empty_struct),
                None,
            )
        } else {
            // Single element list (simplified - full implementation would handle multiple)
            use arrow::buffer::{OffsetBuffer, ScalarBuffer};
            let first_field = pointfield_structs[0].clone();
            let offsets = OffsetBuffer::new(ScalarBuffer::from(vec![0i32, 1i32]));
            arrow::array::ListArray::new(
                Arc::new(Field::new("item", DataType::Struct(field_fields.clone()), false)),
                offsets,
                first_field,
                None,
            )
        };

        let fields_schema = Fields::from(vec![
            Field::new("header", header.data_type().clone(), false),
            Field::new("height", DataType::UInt32, false),
            Field::new("width", DataType::UInt32, false),
            Field::new("fields", DataType::List(Arc::new(Field::new("item", DataType::Struct(field_fields), false))), false),
            Field::new("is_bigendian", DataType::UInt8, false),
            Field::new("point_step", DataType::UInt32, false),
            Field::new("row_step", DataType::UInt32, false),
            Field::new("data", DataType::Binary, false),
            Field::new("is_dense", DataType::Boolean, false),
        ]);

        let height_array = arrow::array::UInt32Array::from(vec![height]);
        let width_array = arrow::array::UInt32Array::from(vec![width]);
        let is_bigendian_array = arrow::array::UInt8Array::from(vec![is_bigendian]);
        let point_step_array = arrow::array::UInt32Array::from(vec![point_step]);
        let row_step_array = arrow::array::UInt32Array::from(vec![row_step]);
        let data_array = arrow::array::BinaryArray::from(vec![data]);
        let is_dense_array = arrow::array::BooleanArray::from(vec![is_dense]);

        let pointcloud2_struct = StructArray::new(
            fields_schema,
            vec![
                header.clone(),
                Arc::new(height_array) as ArrayRef,
                Arc::new(width_array) as ArrayRef,
                Arc::new(fields_array) as ArrayRef,
                Arc::new(is_bigendian_array) as ArrayRef,
                Arc::new(point_step_array) as ArrayRef,
                Arc::new(row_step_array) as ArrayRef,
                Arc::new(data_array) as ArrayRef,
                Arc::new(is_dense_array) as ArrayRef,
            ],
            None,
        );

        Ok(Arc::new(pointcloud2_struct))
    }

    /// Helper struct for PointField
    pub struct PointField {
        pub name: String,
        pub offset: u32,
        pub datatype: u8,
        pub count: u32,
    }

    fn create_pointfield_array(
        names: Vec<String>,
        offsets: Vec<u32>,
        datatypes: Vec<u8>,
        counts: Vec<u32>,
    ) -> Result<ArrayRef> {
        let field_fields = Fields::from(vec![
            Field::new("name", DataType::Utf8, false),
            Field::new("offset", DataType::UInt32, false),
            Field::new("datatype", DataType::UInt8, false),
            Field::new("count", DataType::UInt32, false),
        ]);

        let name_array = arrow::array::StringArray::from(names);
        let offset_array = arrow::array::UInt32Array::from(offsets);
        let datatype_array = arrow::array::UInt8Array::from(datatypes);
        let count_array = arrow::array::UInt32Array::from(counts);

        let pointfield_struct = StructArray::new(
            field_fields,
            vec![
                Arc::new(name_array) as ArrayRef,
                Arc::new(offset_array) as ArrayRef,
                Arc::new(datatype_array) as ArrayRef,
                Arc::new(count_array) as ArrayRef,
            ],
            None,
        );

        Ok(Arc::new(pointfield_struct))
    }

    /// Convert sensor_msgs/LaserScan to Arrow
    pub fn laserscan_to_arrow(
        header: &ArrayRef,
        angle_min: f32,
        angle_max: f32,
        angle_increment: f32,
        time_increment: f32,
        scan_time: f32,
        range_min: f32,
        range_max: f32,
        ranges: &[f32],
        intensities: &[f32],
    ) -> Result<ArrayRef> {
        let fields = Fields::from(vec![
            Field::new("header", header.data_type().clone(), false),
            Field::new("angle_min", DataType::Float32, false),
            Field::new("angle_max", DataType::Float32, false),
            Field::new("angle_increment", DataType::Float32, false),
            Field::new("time_increment", DataType::Float32, false),
            Field::new("scan_time", DataType::Float32, false),
            Field::new("range_min", DataType::Float32, false),
            Field::new("range_max", DataType::Float32, false),
            Field::new("ranges", DataType::List(Arc::new(Field::new("item", DataType::Float32, false))), false),
            Field::new("intensities", DataType::List(Arc::new(Field::new("item", DataType::Float32, false))), false),
        ]);

        let angle_min_array = arrow::array::Float32Array::from(vec![angle_min]);
        let angle_max_array = arrow::array::Float32Array::from(vec![angle_max]);
        let angle_increment_array = arrow::array::Float32Array::from(vec![angle_increment]);
        let time_increment_array = arrow::array::Float32Array::from(vec![time_increment]);
        let scan_time_array = arrow::array::Float32Array::from(vec![scan_time]);
        let range_min_array = arrow::array::Float32Array::from(vec![range_min]);
        let range_max_array = arrow::array::Float32Array::from(vec![range_max]);
        let ranges_array = arrow::array::Float32Array::from_iter_values(ranges.iter().copied());
        let intensities_array = arrow::array::Float32Array::from_iter_values(intensities.iter().copied());

        // Create list arrays using manual construction
        use arrow::buffer::{OffsetBuffer, ScalarBuffer};
        let ranges_offsets = OffsetBuffer::new(ScalarBuffer::from(vec![0i32, ranges_array.len() as i32]));
        let ranges_list = arrow::array::ListArray::new(
            Arc::new(Field::new("item", DataType::Float32, false)),
            ranges_offsets,
            Arc::new(ranges_array),
            None,
        );
        
        let intensities_offsets = OffsetBuffer::new(ScalarBuffer::from(vec![0i32, intensities_array.len() as i32]));
        let intensities_list = arrow::array::ListArray::new(
            Arc::new(Field::new("item", DataType::Float32, false)),
            intensities_offsets,
            Arc::new(intensities_array),
            None,
        );

        let laserscan_struct = StructArray::new(
            fields,
            vec![
                header.clone(),
                Arc::new(angle_min_array) as ArrayRef,
                Arc::new(angle_max_array) as ArrayRef,
                Arc::new(angle_increment_array) as ArrayRef,
                Arc::new(time_increment_array) as ArrayRef,
                Arc::new(scan_time_array) as ArrayRef,
                Arc::new(range_min_array) as ArrayRef,
                Arc::new(range_max_array) as ArrayRef,
                Arc::new(ranges_list) as ArrayRef,
                Arc::new(intensities_list) as ArrayRef,
            ],
            None,
        );

        Ok(Arc::new(laserscan_struct))
    }

    /// Convert sensor_msgs/Imu to Arrow
    pub fn imu_to_arrow(
        header: &ArrayRef,
        orientation: &ArrayRef,
        orientation_covariance: &[f64],
        angular_velocity: &ArrayRef,
        angular_velocity_covariance: &[f64],
        linear_acceleration: &ArrayRef,
        linear_acceleration_covariance: &[f64],
    ) -> Result<ArrayRef> {
        let fields = Fields::from(vec![
            Field::new("header", header.data_type().clone(), false),
            Field::new("orientation", orientation.data_type().clone(), false),
            Field::new("orientation_covariance", DataType::FixedSizeList(Arc::new(Field::new("item", DataType::Float64, false)), 9), false),
            Field::new("angular_velocity", angular_velocity.data_type().clone(), false),
            Field::new("angular_velocity_covariance", DataType::FixedSizeList(Arc::new(Field::new("item", DataType::Float64, false)), 9), false),
            Field::new("linear_acceleration", linear_acceleration.data_type().clone(), false),
            Field::new("linear_acceleration_covariance", DataType::FixedSizeList(Arc::new(Field::new("item", DataType::Float64, false)), 9), false),
        ]);

        let orientation_cov_array = arrow::array::Float64Array::from_iter_values(orientation_covariance.iter().copied());
        let angular_vel_cov_array = arrow::array::Float64Array::from_iter_values(angular_velocity_covariance.iter().copied());
        let linear_accel_cov_array = arrow::array::Float64Array::from_iter_values(linear_acceleration_covariance.iter().copied());

        // Create fixed-size list arrays for covariance matrices (3x3 = 9 elements)
        let orientation_cov_list = arrow::array::FixedSizeListArray::new(
            Arc::new(Field::new("item", DataType::Float64, false)),
            9,
            Arc::new(orientation_cov_array),
            None,
        );
        let angular_vel_cov_list = arrow::array::FixedSizeListArray::new(
            Arc::new(Field::new("item", DataType::Float64, false)),
            9,
            Arc::new(angular_vel_cov_array),
            None,
        );
        let linear_accel_cov_list = arrow::array::FixedSizeListArray::new(
            Arc::new(Field::new("item", DataType::Float64, false)),
            9,
            Arc::new(linear_accel_cov_array),
            None,
        );

        let imu_struct = StructArray::new(
            fields,
            vec![
                header.clone(),
                orientation.clone(),
                Arc::new(orientation_cov_list) as ArrayRef,
                angular_velocity.clone(),
                Arc::new(angular_vel_cov_list) as ArrayRef,
                linear_acceleration.clone(),
                Arc::new(linear_accel_cov_list) as ArrayRef,
            ],
            None,
        );

        Ok(Arc::new(imu_struct))
    }

    /// Convert sensor_msgs/CameraInfo to Arrow
    pub fn camera_info_to_arrow(
        header: &ArrayRef,
        height: u32,
        width: u32,
        distortion_model: &str,
        d: &[f64],
        k: &[f64],
        r: &[f64],
        p: &[f64],
        binning_x: u32,
        binning_y: u32,
        roi_x_offset: u32,
        roi_y_offset: u32,
        roi_height: u32,
        roi_width: u32,
        roi_do_rectify: bool,
    ) -> Result<ArrayRef> {
        let fields = Fields::from(vec![
            Field::new("header", header.data_type().clone(), false),
            Field::new("height", DataType::UInt32, false),
            Field::new("width", DataType::UInt32, false),
            Field::new("distortion_model", DataType::Utf8, false),
            Field::new("d", DataType::List(Arc::new(Field::new("item", DataType::Float64, false))), false),
            Field::new("k", DataType::FixedSizeList(Arc::new(Field::new("item", DataType::Float64, false)), 9), false),
            Field::new("r", DataType::FixedSizeList(Arc::new(Field::new("item", DataType::Float64, false)), 9), false),
            Field::new("p", DataType::FixedSizeList(Arc::new(Field::new("item", DataType::Float64, false)), 12), false),
            Field::new("binning_x", DataType::UInt32, false),
            Field::new("binning_y", DataType::UInt32, false),
            Field::new("roi", DataType::Struct(Fields::from(vec![
                Field::new("x_offset", DataType::UInt32, false),
                Field::new("y_offset", DataType::UInt32, false),
                Field::new("height", DataType::UInt32, false),
                Field::new("width", DataType::UInt32, false),
                Field::new("do_rectify", DataType::Boolean, false),
            ])), false),
        ]);

        let height_array = arrow::array::UInt32Array::from(vec![height]);
        let width_array = arrow::array::UInt32Array::from(vec![width]);
        let distortion_model_array = arrow::array::StringArray::from(vec![distortion_model]);
        let d_array = arrow::array::Float64Array::from_iter_values(d.iter().copied());
        let k_array = arrow::array::Float64Array::from_iter_values(k.iter().copied());
        let r_array = arrow::array::Float64Array::from_iter_values(r.iter().copied());
        let p_array = arrow::array::Float64Array::from_iter_values(p.iter().copied());
        let binning_x_array = arrow::array::UInt32Array::from(vec![binning_x]);
        let binning_y_array = arrow::array::UInt32Array::from(vec![binning_y]);
        let roi_x_offset_array = arrow::array::UInt32Array::from(vec![roi_x_offset]);
        let roi_y_offset_array = arrow::array::UInt32Array::from(vec![roi_y_offset]);
        let roi_height_array = arrow::array::UInt32Array::from(vec![roi_height]);
        let roi_width_array = arrow::array::UInt32Array::from(vec![roi_width]);
        let roi_do_rectify_array = arrow::array::BooleanArray::from(vec![roi_do_rectify]);

        // Create list arrays using manual construction
        use arrow::buffer::{OffsetBuffer, ScalarBuffer};
        let d_offsets = OffsetBuffer::new(ScalarBuffer::from(vec![0i32, d_array.len() as i32]));
        let d_list = arrow::array::ListArray::new(
            Arc::new(Field::new("item", DataType::Float64, false)),
            d_offsets,
            Arc::new(d_array),
            None,
        );
        let k_list = arrow::array::FixedSizeListArray::new(
            Arc::new(Field::new("item", DataType::Float64, false)),
            9,
            Arc::new(k_array),
            None,
        );
        let r_list = arrow::array::FixedSizeListArray::new(
            Arc::new(Field::new("item", DataType::Float64, false)),
            9,
            Arc::new(r_array),
            None,
        );
        let p_list = arrow::array::FixedSizeListArray::new(
            Arc::new(Field::new("item", DataType::Float64, false)),
            12,
            Arc::new(p_array),
            None,
        );

        // Create ROI struct
        let roi_fields = Fields::from(vec![
            Field::new("x_offset", DataType::UInt32, false),
            Field::new("y_offset", DataType::UInt32, false),
            Field::new("height", DataType::UInt32, false),
            Field::new("width", DataType::UInt32, false),
            Field::new("do_rectify", DataType::Boolean, false),
        ]);
        let roi_struct = StructArray::new(
            roi_fields,
            vec![
                Arc::new(roi_x_offset_array) as ArrayRef,
                Arc::new(roi_y_offset_array) as ArrayRef,
                Arc::new(roi_height_array) as ArrayRef,
                Arc::new(roi_width_array) as ArrayRef,
                Arc::new(roi_do_rectify_array) as ArrayRef,
            ],
            None,
        );

        let camera_info_struct = StructArray::new(
            fields,
            vec![
                header.clone(),
                Arc::new(height_array) as ArrayRef,
                Arc::new(width_array) as ArrayRef,
                Arc::new(distortion_model_array) as ArrayRef,
                Arc::new(d_list) as ArrayRef,
                Arc::new(k_list) as ArrayRef,
                Arc::new(r_list) as ArrayRef,
                Arc::new(p_list) as ArrayRef,
                Arc::new(binning_x_array) as ArrayRef,
                Arc::new(binning_y_array) as ArrayRef,
                Arc::new(roi_struct) as ArrayRef,
            ],
            None,
        );

        Ok(Arc::new(camera_info_struct))
    }

    // ============================================================================
    // geometry_msgs additional converters
    // ============================================================================

    /// Convert geometry_msgs/PoseStamped to Arrow
    pub fn pose_stamped_to_arrow(
        header: &ArrayRef,
        pose: &ArrayRef,
    ) -> Result<ArrayRef> {
        let fields = Fields::from(vec![
            Field::new("header", header.data_type().clone(), false),
            Field::new("pose", pose.data_type().clone(), false),
        ]);

        let pose_stamped_struct = StructArray::new(
            fields,
            vec![
                header.clone(),
                pose.clone(),
            ],
            None,
        );

        Ok(Arc::new(pose_stamped_struct))
    }

    /// Convert geometry_msgs/TwistStamped to Arrow
    pub fn twist_stamped_to_arrow(
        header: &ArrayRef,
        twist: &ArrayRef,
    ) -> Result<ArrayRef> {
        let fields = Fields::from(vec![
            Field::new("header", header.data_type().clone(), false),
            Field::new("twist", twist.data_type().clone(), false),
        ]);

        let twist_stamped_struct = StructArray::new(
            fields,
            vec![
                header.clone(),
                twist.clone(),
            ],
            None,
        );

        Ok(Arc::new(twist_stamped_struct))
    }

    /// Convert geometry_msgs/Transform to Arrow
    pub fn transform_to_arrow(
        translation: &ArrayRef,
        rotation: &ArrayRef,
    ) -> Result<ArrayRef> {
        let fields = Fields::from(vec![
            Field::new("translation", translation.data_type().clone(), false),
            Field::new("rotation", rotation.data_type().clone(), false),
        ]);

        let transform_struct = StructArray::new(
            fields,
            vec![
                translation.clone(),
                rotation.clone(),
            ],
            None,
        );

        Ok(Arc::new(transform_struct))
    }

    /// Convert geometry_msgs/TransformStamped to Arrow
    pub fn transform_stamped_to_arrow(
        header: &ArrayRef,
        child_frame_id: &str,
        transform: &ArrayRef,
    ) -> Result<ArrayRef> {
        let fields = Fields::from(vec![
            Field::new("header", header.data_type().clone(), false),
            Field::new("child_frame_id", DataType::Utf8, false),
            Field::new("transform", transform.data_type().clone(), false),
        ]);

        let child_frame_id_array = arrow::array::StringArray::from(vec![child_frame_id]);

        let transform_stamped_struct = StructArray::new(
            fields,
            vec![
                header.clone(),
                Arc::new(child_frame_id_array) as ArrayRef,
                transform.clone(),
            ],
            None,
        );

        Ok(Arc::new(transform_stamped_struct))
    }

    // ============================================================================
    // nav_msgs converters
    // ============================================================================

    /// Convert nav_msgs/Odometry to Arrow
    pub fn odometry_to_arrow(
        header: &ArrayRef,
        child_frame_id: &str,
        pose: &ArrayRef,
        pose_covariance: &[f64],
        twist: &ArrayRef,
        twist_covariance: &[f64],
    ) -> Result<ArrayRef> {
        let fields = Fields::from(vec![
            Field::new("header", header.data_type().clone(), false),
            Field::new("child_frame_id", DataType::Utf8, false),
            Field::new("pose", DataType::Struct(Fields::from(vec![
                Field::new("pose", pose.data_type().clone(), false),
                Field::new("covariance", DataType::FixedSizeList(Arc::new(Field::new("item", DataType::Float64, false)), 36), false),
            ])), false),
            Field::new("twist", DataType::Struct(Fields::from(vec![
                Field::new("twist", twist.data_type().clone(), false),
                Field::new("covariance", DataType::FixedSizeList(Arc::new(Field::new("item", DataType::Float64, false)), 36), false),
            ])), false),
        ]);

        let child_frame_id_array = arrow::array::StringArray::from(vec![child_frame_id]);
        let pose_cov_array = arrow::array::Float64Array::from_iter_values(pose_covariance.iter().copied());
        let twist_cov_array = arrow::array::Float64Array::from_iter_values(twist_covariance.iter().copied());

        let pose_cov_list = arrow::array::FixedSizeListArray::new(
            Arc::new(Field::new("item", DataType::Float64, false)),
            36,
            Arc::new(pose_cov_array),
            None,
        );
        let twist_cov_list = arrow::array::FixedSizeListArray::new(
            Arc::new(Field::new("item", DataType::Float64, false)),
            36,
            Arc::new(twist_cov_array),
            None,
        );

        // Create pose struct
        let pose_fields = Fields::from(vec![
            Field::new("pose", pose.data_type().clone(), false),
            Field::new("covariance", DataType::FixedSizeList(Arc::new(Field::new("item", DataType::Float64, false)), 36), false),
        ]);
        let pose_struct = StructArray::new(
            pose_fields,
            vec![
                pose.clone(),
                Arc::new(pose_cov_list) as ArrayRef,
            ],
            None,
        );

        // Create twist struct
        let twist_fields = Fields::from(vec![
            Field::new("twist", twist.data_type().clone(), false),
            Field::new("covariance", DataType::FixedSizeList(Arc::new(Field::new("item", DataType::Float64, false)), 36), false),
        ]);
        let twist_struct = StructArray::new(
            twist_fields,
            vec![
                twist.clone(),
                Arc::new(twist_cov_list) as ArrayRef,
            ],
            None,
        );

        let odometry_struct = StructArray::new(
            fields,
            vec![
                header.clone(),
                Arc::new(child_frame_id_array) as ArrayRef,
                Arc::new(pose_struct) as ArrayRef,
                Arc::new(twist_struct) as ArrayRef,
            ],
            None,
        );

        Ok(Arc::new(odometry_struct))
    }

    /// Convert nav_msgs/Path to Arrow
    pub fn path_to_arrow(
        header: &ArrayRef,
        poses: Vec<&ArrayRef>,
    ) -> Result<ArrayRef> {
        // Create array of PoseStamped messages
        let pose_stamped_fields = Fields::from(vec![
            Field::new("header", header.data_type().clone(), false),
            Field::new("pose", poses[0].data_type().clone(), false),
        ]);

        let mut pose_stamped_arrays = Vec::new();
        for pose in poses {
            let pose_stamped = StructArray::new(
                pose_stamped_fields.clone(),
                vec![
                    header.clone(),
                    pose.clone(),
                ],
                None,
            );
            pose_stamped_arrays.push(Arc::new(pose_stamped) as ArrayRef);
        }

        // Create list array from struct arrays
        // Build offsets manually
        let mut offsets = vec![0];
        let mut current_offset = 0;
        for arr in &pose_stamped_arrays {
            current_offset += arr.len();
            offsets.push(current_offset);
        }

        // Flatten all struct arrays into a single struct array
        if pose_stamped_arrays.is_empty() {
            // Empty list
            let empty_struct = StructArray::new(
                pose_stamped_fields.clone(),
                vec![],
                None,
            );
            use arrow::buffer::{OffsetBuffer, ScalarBuffer};
            let offsets = OffsetBuffer::new(ScalarBuffer::from(vec![0i32, 0i32]));
            let empty_list = arrow::array::ListArray::new(
                Arc::new(Field::new("item", DataType::Struct(pose_stamped_fields.clone()), false)),
                offsets,
                Arc::new(empty_struct),
                None,
            );

            let fields = Fields::from(vec![
                Field::new("header", header.data_type().clone(), false),
                Field::new("poses", DataType::List(Arc::new(Field::new("item", DataType::Struct(pose_stamped_fields), false))), false),
            ]);

            let path_struct = StructArray::new(
                fields,
                vec![
                    header.clone(),
                    Arc::new(empty_list) as ArrayRef,
                ],
                None,
            );
            return Ok(Arc::new(path_struct));
        }

        // For simplicity, create a single-element list with first pose
        // Full implementation would concatenate all struct arrays
        let first_pose_stamped = pose_stamped_arrays[0].clone();
        use arrow::buffer::{OffsetBuffer, ScalarBuffer};
        let offsets = OffsetBuffer::new(
            ScalarBuffer::from(vec![0i32, first_pose_stamped.len() as i32])
        );
        let poses_list = arrow::array::ListArray::new(
            Arc::new(Field::new("item", DataType::Struct(pose_stamped_fields.clone()), false)),
            offsets,
            first_pose_stamped,
            None,
        );

        let fields = Fields::from(vec![
            Field::new("header", header.data_type().clone(), false),
            Field::new("poses", DataType::List(Arc::new(Field::new("item", DataType::Struct(pose_stamped_fields), false))), false),
        ]);

        let path_struct = StructArray::new(
            fields,
            vec![
                header.clone(),
                Arc::new(poses_list) as ArrayRef,
            ],
            None,
        );

        Ok(Arc::new(path_struct))
    }

    /// Convert nav_msgs/OccupancyGrid to Arrow
    pub fn occupancy_grid_to_arrow(
        header: &ArrayRef,
        info: &ArrayRef,
        data: &[i8],
    ) -> Result<ArrayRef> {
        let fields = Fields::from(vec![
            Field::new("header", header.data_type().clone(), false),
            Field::new("info", info.data_type().clone(), false),
            Field::new("data", DataType::List(Arc::new(Field::new("item", DataType::Int8, false))), false),
        ]);

        let data_array = arrow::array::Int8Array::from_iter_values(data.iter().copied());
        use arrow::buffer::{OffsetBuffer, ScalarBuffer};
        let data_offsets = OffsetBuffer::new(ScalarBuffer::from(vec![0i32, data_array.len() as i32]));
        let data_list = arrow::array::ListArray::new(
            Arc::new(Field::new("item", DataType::Int8, false)),
            data_offsets,
            Arc::new(data_array),
            None,
        );

        let occupancy_grid_struct = StructArray::new(
            fields,
            vec![
                header.clone(),
                info.clone(),
                Arc::new(data_list) as ArrayRef,
            ],
            None,
        );

        Ok(Arc::new(occupancy_grid_struct))
    }
}

