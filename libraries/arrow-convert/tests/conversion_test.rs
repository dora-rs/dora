use std::sync::Arc;
use std::convert::TryFrom;
use dora_arrow_convert::{ArrowData, IntoArrow};

#[cfg(test)]
mod tests {
    use super::*;
    use eyre::Report;

    #[test]
    fn test_bool_round_trip() -> Result<(), Report> {
        let value_bool: bool = true;
        let arrow_array = value_bool.into_arrow();
        let data: ArrowData = ArrowData(Arc::new(arrow_array));
        let result_bool: bool = TryFrom::try_from(&data)?;
        assert_eq!(value_bool, result_bool);
        Ok(())
    }


    #[test]
    fn test_u8_round_trip() -> Result<(), Report> {
        let value_u8: u8 = 42;
        let arrow_array = value_u8.into_arrow();
        let data: ArrowData = ArrowData(Arc::new(arrow_array));
        let result_u8: u8 = TryFrom::try_from(&data)?;
        assert_eq!(value_u8, result_u8);
        Ok(())
    }

    #[test]
    fn test_u16_round_trip() -> Result<(), Report> {
        let value_u16: u16 = 42;
        let arrow_array = value_u16.into_arrow();
        let data: ArrowData = ArrowData(Arc::new(arrow_array));
        let result_u16: u16 = TryFrom::try_from(&data)?;
        assert_eq!(value_u16, result_u16);
        Ok(())
    }

    #[test]
    fn test_u32_round_trip() -> Result<(), Report> {
        let value_u32: u32 = 42;
        let arrow_array = value_u32.into_arrow();
        let data: ArrowData = ArrowData(Arc::new(arrow_array));
        let result_u32: u32 = TryFrom::try_from(&data)?;
        assert_eq!(value_u32, result_u32);
        Ok(())
    }

    #[test]
    fn test_u64_round_trip() -> Result<(), Report> {
        let value_u64: u64 = 42;
        let arrow_array = value_u64.into_arrow();
        let data: ArrowData = ArrowData(Arc::new(arrow_array));
        let result_u64: u64 = TryFrom::try_from(&data)?;
        assert_eq!(value_u64, result_u64);
        Ok(())
    }

    #[test]
    fn test_i8_round_trip() -> Result<(), Report> {
        let value_i8: i8 = 42;
        let arrow_array = value_i8.into_arrow();
        let data: ArrowData = ArrowData(Arc::new(arrow_array));
        let result_i8: i8 = TryFrom::try_from(&data)?;
        assert_eq!(value_i8, result_i8);
        Ok(())
    }

    #[test]
    fn test_i16_round_trip() -> Result<(), Report> {
        let value_i16: i16 = 42;
        let arrow_array = value_i16.into_arrow();
        let data: ArrowData = ArrowData(Arc::new(arrow_array));
        let result_i16: i16 = TryFrom::try_from(&data)?;
        assert_eq!(value_i16, result_i16);
        Ok(())
    }

    #[test]
    fn test_i32_round_trip() -> Result<(), Report> {
        let value_i32: i32 = 42;
        let arrow_array = value_i32.into_arrow();
        let data: ArrowData = ArrowData(Arc::new(arrow_array));
        let result_i32: i32 = TryFrom::try_from(&data)?;
        assert_eq!(value_i32, result_i32);
        Ok(())
    }

    #[test]
    fn test_i64_round_trip() -> Result<(), Report> {
        let value_i64: i64 = 42;
        let arrow_array = value_i64.into_arrow();
        let data: ArrowData = ArrowData(Arc::new(arrow_array));
        let result_i64: i64 = TryFrom::try_from(&data)?;
        assert_eq!(value_i64, result_i64);
        Ok(())
    }

    #[test]
    fn test_f32_round_trip() -> Result<(), Report> {
        let value_f32: f32 = 42.42;
        let arrow_array = value_f32.into_arrow();
        let data: ArrowData = ArrowData(Arc::new(arrow_array));
        let result_f32: f32 = TryFrom::try_from(&data)?;
        assert_eq!(value_f32, result_f32);
        Ok(())
    }

    #[test]
    fn test_f64_round_trip() -> Result<(), Report> {
        let value_f64: f64 = 42.42;
        let arrow_array = value_f64.into_arrow();
        let data: ArrowData = ArrowData(Arc::new(arrow_array));
        let result_f64: f64 = TryFrom::try_from(&data)?;
        assert_eq!(value_f64, result_f64);
        Ok(())
    }

    #[test]
    fn test_str_round_trip() -> Result<(), Report> {
        let value_str: &str = "Hello, Arrow!";
        let arrow_array = value_str.into_arrow();
        let data: ArrowData = ArrowData(Arc::new(arrow_array));
        let result_str: &str = TryFrom::try_from(&data)?;
        assert_eq!(value_str, result_str);
        Ok(())
    }

    #[test]
    fn test_vec_u8_round_trip() -> Result<(), Report> {
        let value_vec_u8: Vec<u8> = vec![1, 2, 3, 4, 5];
        let arrow_array = value_vec_u8.clone().into_arrow();
        let data: ArrowData = ArrowData(Arc::new(arrow_array));
        let result_vec_u8: Vec<u8> = TryFrom::try_from(&data)?;
        assert_eq!(value_vec_u8, result_vec_u8);
        Ok(())
    }


    // #[test]
    // fn test_vec_u16_round_trip() -> Result<(), Report> {
    //     let value_vec_u16: Vec<u16> = vec![1, 2, 3, 4, 5];
    //     let arrow_array = value_vec_u16.clone().into_arrow();
    //     let data: ArrowData = ArrowData(Arc::new(arrow_array));
    //     let result_vec_u16: Vec<u16> = TryFrom::try_from(&data)?;
    //     assert_eq!(value_vec_u16, result_vec_u16);
    //     Ok(())
    // }

    // #[test]
    // fn test_vec_u32_round_trip() -> Result<(), Report> {
    //     let value_vec_u32: Vec<u32> = vec![1, 2, 3, 4, 5];
    //     let arrow_array = value_vec_u32.clone().into_arrow();
    //     let data: ArrowData = ArrowData(Arc::new(arrow_array));
    //     let result_vec_u32: Vec<u32> = TryFrom::try_from(&data)?;
    //     assert_eq!(value_vec_u32, result_vec_u32);
    //     Ok(())
    // }

    // #[test]
    // fn test_vec_u64_round_trip() -> Result<(), Report> {
    //     let value_vec_u64: Vec<u64> = vec![1, 2, 3, 4, 5];
    //     let arrow_array = value_vec_u64.clone().into_arrow();
    //     let data: ArrowData = ArrowData(Arc::new(arrow_array));
    //     let result_vec_u64: Vec<u64> = TryFrom::try_from(&data)?;
    //     assert_eq!(value_vec_u64, result_vec_u64);
    //     Ok(())
    // }

    
    // #[test]
    // fn test_vec_i8_round_trip() -> Result<(), Report> {
    //     let value_vec_i8: Vec<i8> = vec![-1, -2, -3, -4, -5];
    //     let arrow_array = value_vec_i8.clone().into_arrow();
    //     let data: ArrowData = ArrowData(Arc::new(arrow_array));
    //     let result_vec_i8: Vec<i8> = TryFrom::try_from(&data)?;
    //     assert_eq!(value_vec_i8, result_vec_i8);
    //     Ok(())
    // }

    // #[test]
    // fn test_vec_i16_round_trip() -> Result<(), Report> {
    //     let value_vec_i16: Vec<i16> = vec![-1, -2, -3, -4, -5];
    //     let arrow_array = value_vec_i16.clone().into_arrow();
    //     let data: ArrowData = ArrowData(Arc::new(arrow_array));
    //     let result_vec_i16: Vec<i16> = TryFrom::try_from(&data)?;
    //     assert_eq!(value_vec_i16, result_vec_i16);
    //     Ok(())
    // }

    // #[test]
    // fn test_vec_i32_round_trip() -> Result<(), Report> {
    //     let value_vec_i32: Vec<i32> = vec![-1, -2, -3, -4, -5];
    //     let arrow_array = value_vec_i32.clone().into_arrow();
    //     let data: ArrowData = ArrowData(Arc::new(arrow_array));
    //     let result_vec_i32: Vec<i32> = TryFrom::try_from(&data)?;
    //     assert_eq!(value_vec_i32, result_vec_i32);
    //     Ok(())
    // }

    // #[test]
    // fn test_vec_i64_round_trip() -> Result<(), Report> {
    //     let value_vec_i64: Vec<i64> = vec![-1, -2, -3, -4, -5];
    //     let arrow_array = value_vec_i64.clone().into_arrow();
    //     let data: ArrowData = ArrowData(Arc::new(arrow_array));
    //     let result_vec_i64: Vec<i64> = TryFrom::try_from(&data)?;
    //     assert_eq!(value_vec_i64, result_vec_i64);
    //     Ok(())
    // }

}

