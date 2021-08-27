use std::{
    collections::HashMap,
    sync::{Arc, Mutex},
};

use anyhow::Result;

use super::{
    Parameter, ParameterDescriptor, ParameterType, ParameterValue, RclParams, SetParametersResult,
};
use crate::{context::RclContext, error::RclRustError, node::RclNode};

#[derive(Debug, Default, Clone)]
pub struct ParameterInfo {
    value: ParameterValue,
    descriptor: ParameterDescriptor,
}

impl ParameterInfo {
    fn get_type(&self) -> ParameterType {
        self.value.get_type()
    }
}

#[derive(Debug, Default)]
pub struct Parameters {
    parameters: Mutex<HashMap<String, ParameterInfo>>,
    parameter_overrides: HashMap<String, ParameterValue>,
    allow_undeclared: bool,
}

impl Parameters {
    pub(crate) fn new(
        context_handle: Arc<Mutex<RclContext>>,
        node_handle: &RclNode,
    ) -> Result<Self> {
        let mut parameter_overrides = HashMap::new();

        if node_handle.use_global_arguments().unwrap() {
            if let Some(rcl_params) =
                RclParams::new(context_handle.lock().unwrap().global_arguments())?
            {
                parameter_overrides =
                    rcl_params.to_parameters(&node_handle.fully_qualified_name())?;
            }
        }
        Ok(Self {
            parameter_overrides,
            ..Default::default()
        })
    }

    pub fn declare_parameter(
        &self,
        name: &str,
        default_value: &ParameterValue,
        parameter_descriptor: ParameterDescriptor,
        ignore_override: bool,
    ) -> Result<()> {
        let mut params = self.parameters.lock().unwrap();

        if name.is_empty() {
            return Err(
                RclRustError::ParameterInvalid("parameter name must not be empty".into()).into(),
            );
        }

        if params.contains_key(name) {
            return Err(RclRustError::ParameterAlreadyDeclared { name: name.into() }.into());
        }

        if let Err(reason) = declare_parameter_common(
            name,
            default_value,
            parameter_descriptor,
            &mut params,
            &self.parameter_overrides,
            ignore_override,
        ) {
            return Err(RclRustError::ParameterInvalidValue {
                name: name.into(),
                reason,
            }
            .into());
        }

        Ok(())
    }

    pub fn has_parameter(&self, name: &str) -> bool {
        self.parameters.lock().unwrap().contains_key(name)
    }

    pub fn get_parameter(&self, name: &str) -> Option<Parameter> {
        if let Some(info) = self.parameters.lock().unwrap().get(name) {
            if info.get_type() != ParameterType::NotSet {
                return Some(Parameter {
                    name: name.into(),
                    value: info.value.clone(),
                });
            }
        }

        if self.allow_undeclared {
            Some(Parameter {
                name: name.into(),
                value: ParameterValue::not_set(),
            })
        } else {
            None
        }
    }

    // pub fn set_parameters(&self, parameters: &[Parameter]) -> Result<Vec<SetParametersResult>> {
    //     parameters
    //         .iter()
    //         .map(|p| self.set_parameters_atomically(&[p.clone()]))
    //         .collect::<Result<Vec<_>>>()
    // }

    pub fn set_parameters_atomically(
        &self,
        parameters: &[Parameter],
    ) -> Result<SetParametersResult> {
        let mut params = self.parameters.lock().unwrap();
        let mut parameters_to_be_declared = Vec::new();

        for parameter in parameters {
            if parameter.name.is_empty() {
                return Err(RclRustError::ParameterInvalid(
                    "Parameter name must not be empty".into(),
                )
                .into());
            }

            match params.get(&parameter.name) {
                Some(info) => {
                    if info.descriptor.read_only {
                        return Ok(SetParametersResult {
                            successful: false,
                            reason: format!(
                                "Parameter {{{}}} cannot be set because it is read-only.",
                                parameter.name
                            ),
                        });
                    }
                }
                None => {
                    if self.allow_undeclared {
                        parameters_to_be_declared.push(parameter);
                    } else {
                        return Err(RclRustError::ParameterNotDeclared {
                            name: parameter.name.clone(),
                        }
                        .into());
                    }
                }
            }
        }

        let mut staged_parameter_changes = HashMap::<String, ParameterInfo>::new();
        for param in parameters_to_be_declared {
            if let Err(reason) = declare_parameter_common(
                &param.name,
                &param.value,
                ParameterDescriptor::default(),
                &mut staged_parameter_changes,
                &self.parameter_overrides,
                true,
            ) {
                return Ok(SetParametersResult {
                    successful: false,
                    reason,
                });
            }
        }

        let parameters_to_be_undeclared = parameters
            .iter()
            .filter(|param| {
                params
                    .get(&param.name)
                    .map(|v| v.get_type() == ParameterType::NotSet)
                    .unwrap_or(false)
            })
            .collect::<Vec<_>>();

        if let Err(reason) = set_parameters_atomically_common(parameters, &mut params) {
            return Ok(SetParametersResult {
                successful: false,
                reason,
            });
        }

        for (change_name, change_info) in &staged_parameter_changes {
            assert!(params.contains_key(change_name));
            assert!(params.get(change_name).unwrap().value == change_info.value);
            params.insert(change_name.clone(), change_info.clone());
        }

        for param in parameters_to_be_undeclared {
            params.remove(&param.name).unwrap();
        }

        Ok(SetParametersResult {
            successful: true,
            reason: "".into(),
        })
    }
}

fn set_parameters_atomically_common(
    parameters: &[Parameter],
    parameters_info: &mut HashMap<String, ParameterInfo>,
) -> std::result::Result<(), String> {
    for param in parameters {
        param.value.check_range(
            &parameters_info
                .entry(param.name.clone())
                .or_insert_with(Default::default)
                .descriptor,
        )?;
    }

    for param in parameters {
        let info = parameters_info.get_mut(&param.name).unwrap();
        info.descriptor.name = param.name.clone();
        info.descriptor.type_ = param.value.get_u8_type();
        info.value = param.value.clone();
    }

    Ok(())
}

fn declare_parameter_common(
    name: &str,
    default_value: &ParameterValue,
    parameter_descriptor: ParameterDescriptor,
    parameters_out: &mut HashMap<String, ParameterInfo>,
    overrides: &HashMap<String, ParameterValue>,
    ignore_override: bool,
) -> std::result::Result<(), String> {
    let mut parameter_infos = HashMap::new();
    parameter_infos.insert(
        name.into(),
        ParameterInfo {
            value: Default::default(),
            descriptor: parameter_descriptor,
        },
    );

    let mut initial_value = default_value;
    if !ignore_override {
        if let Some(override_) = overrides.get(name) {
            initial_value = override_;
        }
    }

    set_parameters_atomically_common(
        &[Parameter {
            name: name.into(),
            value: initial_value.clone(),
        }],
        &mut parameter_infos,
    )?;

    parameters_out.insert(name.into(), parameter_infos.remove(name).unwrap());

    Ok(())
}

#[cfg(test)]
mod test {
    use super::*;

    const PARAM_NAME: &str = "param1";

    #[test]
    fn declare_and_read_default_value() -> Result<()> {
        let parameters = Parameters::default();

        let default_value = ParameterValue::integer(42);

        parameters.declare_parameter(
            PARAM_NAME,
            &default_value,
            ParameterDescriptor::default(),
            false,
        )?;

        assert!(parameters.has_parameter(PARAM_NAME));
        assert_eq!(
            parameters.get_parameter(PARAM_NAME),
            Some(Parameter::integer(PARAM_NAME, 42))
        );

        Ok(())
    }

    #[test]
    fn read_overrided_value() -> Result<()> {
        let mut parameter_overrides = HashMap::new();
        parameter_overrides.insert(PARAM_NAME.into(), ParameterValue::double(3.14));
        let parameters = Parameters {
            parameter_overrides,
            ..Default::default()
        };

        parameters.declare_parameter(
            PARAM_NAME,
            &ParameterValue::integer(42),
            ParameterDescriptor::default(),
            false,
        )?;

        assert!(parameters.has_parameter(PARAM_NAME));
        assert_eq!(
            parameters.get_parameter(PARAM_NAME),
            Some(Parameter::double(PARAM_NAME, 3.14))
        );

        Ok(())
    }

    #[test]
    fn set_parameter() -> Result<()> {
        let mut parameter_overrides = HashMap::new();
        parameter_overrides.insert(PARAM_NAME.into(), ParameterValue::double(3.14));
        let parameters = Parameters {
            parameter_overrides,
            ..Default::default()
        };

        parameters.declare_parameter(
            PARAM_NAME,
            &ParameterValue::integer(42),
            ParameterDescriptor::default(),
            false,
        )?;

        parameters.set_parameters_atomically(&[Parameter::string(PARAM_NAME, "hoge")])?;
        assert!(parameters.has_parameter(PARAM_NAME));
        assert_eq!(
            parameters.get_parameter(PARAM_NAME),
            Some(Parameter::string(PARAM_NAME, "hoge"))
        );

        Ok(())
    }

    #[test]
    fn check_integer_range() -> Result<()> {
        let parameters = Parameters {
            ..Default::default()
        };

        use rclrust_msg::rcl_interfaces::msg::IntegerRange;

        parameters.declare_parameter(
            PARAM_NAME,
            &ParameterValue::integer(0),
            ParameterDescriptor {
                integer_range: vec![IntegerRange {
                    from_value: -1,
                    to_value: 1,
                    step: 0,
                }],
                ..Default::default()
            },
            false,
        )?;

        assert!(
            parameters
                .set_parameters_atomically(&[Parameter::integer(PARAM_NAME, 1)])?
                .successful
        );
        assert!(
            !parameters
                .set_parameters_atomically(&[Parameter::integer(PARAM_NAME, 5)])?
                .successful
        );

        assert!(parameters.has_parameter(PARAM_NAME));

        Ok(())
    }

    #[test]
    fn set_duplicated_parameters() -> Result<()> {
        let parameters = Parameters {
            allow_undeclared: true,
            ..Default::default()
        };

        parameters.set_parameters_atomically(&[
            Parameter::string(PARAM_NAME, "hoge"),
            Parameter::string(PARAM_NAME, "fuga"),
        ])?;
        assert!(parameters.has_parameter(PARAM_NAME));
        assert_eq!(
            parameters.get_parameter(PARAM_NAME),
            Some(Parameter::string(PARAM_NAME, "fuga"))
        );

        Ok(())
    }

    #[test]
    fn get_not_declared_param() -> Result<()> {
        let parameters = Parameters::default();
        assert!(!parameters.has_parameter("not_exist"));
        assert!(parameters.get_parameter("not_exist").is_none());

        Ok(())
    }

    #[test]
    fn get_not_declared_param_when_allow_undeclared() -> Result<()> {
        let parameters = Parameters {
            allow_undeclared: true,
            ..Default::default()
        };
        assert!(!parameters.has_parameter("not_exist"));
        assert_eq!(
            parameters.get_parameter("not_exist"),
            Some(Parameter::not_set("not_exist"))
        );

        Ok(())
    }
}
