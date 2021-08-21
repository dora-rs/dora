use std::collections::HashMap;

use anyhow::{Context, Result};

use super::ParameterValue;
use crate::{
    error::{RclRustError, ToRclRustResult},
    internal::ffi::{FromCChar, SizedFromCChar},
};

#[derive(Debug)]
pub struct RclParams(Box<*mut rcl_sys::rcl_params_t>);

impl RclParams {
    pub fn new(arguments: &rcl_sys::rcl_arguments_t) -> Result<Option<Self>> {
        let mut params = Box::new(std::ptr::null_mut());
        unsafe {
            rcl_sys::rcl_arguments_get_param_overrides(arguments, &mut *params)
                .to_result()
                .with_context(|| "rcl_sys::rcl_arguments_get_param_overrides in RclParams::new")?;
        }
        if params.is_null() {
            Ok(None)
        } else {
            Ok(Some(Self(params)))
        }
    }

    pub fn to_parameters(
        &self,
        node_fully_qualified_name: &str,
    ) -> Result<HashMap<String, ParameterValue>> {
        if self.0.is_null() {
            return Err(
                RclRustError::ParameterInvalid("Parameter struct is null pointer".into()).into(),
            );
        }
        let rcl_params = unsafe { &**self.0 };

        if rcl_params.node_names.is_null() {
            return Err(
                RclRustError::ParameterInvalid("Node names array is null pointer".into()).into(),
            );
        } else if rcl_params.params.is_null() {
            return Err(
                RclRustError::ParameterInvalid("Node params array is null pointer".into()).into(),
            );
        }

        let node_names =
            unsafe { std::slice::from_raw_parts(rcl_params.node_names, rcl_params.num_nodes) };
        let params = unsafe { std::slice::from_raw_parts(rcl_params.params, rcl_params.num_nodes) };

        let mut parameters = HashMap::new();

        for (node_i, (node_name, param)) in node_names.iter().zip(params).enumerate() {
            if node_name.is_null() {
                return Err(RclRustError::ParameterInvalid(format!(
                    "Node name at index {} is null pointer.",
                    node_i
                ))
                .into());
            }

            let node_name = unsafe { str::from_c_char(*node_name) }.unwrap_or_default();
            let node_name = if !node_name.starts_with('/') {
                format!("/{}", node_name)
            } else {
                node_name.into()
            };

            if node_name != "/**" && node_name != node_fully_qualified_name {
                continue;
            }

            let param_names =
                unsafe { std::slice::from_raw_parts(param.parameter_names, param.num_params) };
            let param_values =
                unsafe { std::slice::from_raw_parts(param.parameter_values, param.num_params) };

            for (param_i, (param_name, param_value)) in
                param_names.iter().zip(param_values).enumerate()
            {
                if param_name.is_null() {
                    return Err(RclRustError::ParameterInvalid(format!(
                        "At node {} parameter {} name is null pointer",
                        node_i, param_i
                    ))
                    .into());
                }

                parameters.insert(
                    unsafe { String::from_c_char(*param_name) }.unwrap_or_default(),
                    ParameterValue::from(param_value),
                );
            }
        }

        Ok(parameters)
    }
}

impl Drop for RclParams {
    fn drop(&mut self) {
        unsafe { rcl_sys::rcl_yaml_node_struct_fini(*self.0) }
    }
}

#[cfg(test)]
mod test {
    use super::*;
    use crate::{context::Context, InitOptions};

    #[test]
    fn rcl_param_new() -> Result<()> {
        let args = "--ros-args -p param_int:=42"
            .split(' ')
            .map(String::from)
            .collect();
        let ctx = Context::new(args, InitOptions::new()?)?;

        assert!(RclParams::new(ctx.handle.lock().unwrap().global_arguments())?.is_some());

        Ok(())
    }

    #[test]
    fn rcl_param_new_from_empty_args() -> Result<()> {
        let ctx = Context::new(Vec::new(), InitOptions::new()?)?;
        assert!(RclParams::new(ctx.handle.lock().unwrap().global_arguments())?.is_none());
        Ok(())
    }

    #[test]
    fn rcl_param_to_parameters() -> Result<()> {
        let args = r#"--ros-args -p param_int:=42 -p param_string:="hello""#
            .split(' ')
            .map(String::from)
            .collect();
        let ctx = Context::new(args, InitOptions::new()?)?;
        let node = ctx.create_node("test_node")?;

        let rcl_params = RclParams::new(ctx.handle.lock().unwrap().global_arguments())?.unwrap();
        let parameters = rcl_params.to_parameters(&node.fully_qualified_name())?;

        assert_eq!(
            parameters.get("param_int"),
            Some(&ParameterValue::integer(42))
        );
        assert_eq!(
            parameters.get("param_string"),
            Some(&ParameterValue::string("hello"))
        );

        Ok(())
    }

    #[test]
    fn rcl_param_to_parameters_array_param() -> Result<()> {
        let args = ["--ros-args", "-p", "doubles:=[3.0, 4.2, -1.2]"]
            .iter()
            .map(|s| s.to_string())
            .collect();
        let ctx = Context::new(args, InitOptions::new()?)?;
        let node = ctx.create_node("test_node")?;

        let rcl_params = RclParams::new(ctx.handle.lock().unwrap().global_arguments())?.unwrap();
        let parameters = rcl_params.to_parameters(&node.fully_qualified_name())?;

        assert_eq!(
            parameters.get("doubles"),
            Some(&ParameterValue::double_array(vec![3.0, 4.2, -1.2]))
        );

        Ok(())
    }
}
