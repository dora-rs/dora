use std::{collections::HashMap, path::PathBuf};

use eyre::{Context, ContextCompat, Result};
use k::{Chain, Translation3, UnitQuaternion, nalgebra::Quaternion};
use rerun::{RecordingStream, Vec3D};
pub struct MyIntersperse<T, I> {
    iterator: I,
    sep: T,
    nxt: Option<T>,
}

pub trait MyIntersperseExt<T: Clone, I: Iterator>: Iterator<Item = T> {
    fn my_intersperse(self, sep: T) -> MyIntersperse<T, I>;
}

impl<T: Clone, I: Iterator<Item = T>> MyIntersperseExt<T, I> for I {
    fn my_intersperse(mut self, sep: T) -> MyIntersperse<T, I> {
        let next = self.next();
        MyIntersperse {
            iterator: self,
            sep: sep.clone(),
            nxt: next,
        }
    }
}

impl<T: Clone, I: Iterator<Item = T>> Iterator for MyIntersperse<T, I> {
    type Item = T;
    fn next(&mut self) -> Option<Self::Item> {
        if let Some(item) = self.nxt.take() {
            Some(item)
        } else {
            self.nxt = self.iterator.next();
            if self.nxt.is_some() {
                Some(self.sep.clone())
            } else {
                None
            }
        }
    }
}

fn get_entity_path(link: &k::Node<f32>, urdf_path: &str) -> String {
    let mut ancestors: Vec<_> = link
        .iter_ancestors()
        .map(|node| node.link().as_ref().unwrap().name.clone())
        .collect();
    ancestors.push(String::from(urdf_path));
    ancestors
        .into_iter()
        .rev()
        .my_intersperse(String::from("/"))
        .collect()
}

pub fn init_urdf(rec: &RecordingStream) -> Result<HashMap<String, Chain<f32>>> {
    // Get all env variable that end with urdf
    let urdfs = std::env::vars()
        .filter(|(key, _)| key.ends_with("_urdf") || key.ends_with("_URDF"))
        .collect::<Vec<_>>();
    let mut chains = HashMap::new();
    for (key, urdf_path) in urdfs {
        let urdf_path = if urdf_path.ends_with("_description") {
            // Use robot description to get its path
            let bash_cmd = format!("robot_descriptions pull {urdf_path}");
            let response = std::process::Command::new("bash")
                .arg("-c")
                .arg(bash_cmd)
                .output()
                .context("Failed to execute bash command. Are you sure `robot_descriptions` is installed?")?
                .stdout;
            let response_str =
                String::from_utf8(response).context("Could not parse robot descriptions")?;
            // Only keep last line of the response
            let response_str = response_str
                .lines()
                .last()
                .context("Could not find last line in robot descriptions response")?;
            PathBuf::from(response_str.trim())
        } else {
            // Use the path directly
            PathBuf::from(urdf_path)
        };
        let chain = k::Chain::<f32>::from_urdf_file(&urdf_path)
            .context(format!("Could not load URDF {urdf_path:#?}"))?;

        let path = key.replace("_urdf", ".urdf").replace("_URDF", ".urdf");
        let transform = key.replace("_urdf", "_transform");

        if PathBuf::from(&urdf_path).file_name() != PathBuf::from(&path).file_name() {
            return Err(eyre::eyre!(
                "URDF filename should be the same as the environment variable name and replacing the dot with a dash. Got {:#?} instead of {}",
                urdf_path,
                path
            ));
        }
        rec.log_file_from_path(&urdf_path, None, true)
            .context(format!(
                "Could not log URDF file {urdf_path:#?} within rerun-urdf-loader"
            ))?;
        println!("Logging URDF file: {urdf_path:#?}");

        // Get transform by replacing URDF_ with TRANSFORM_
        if let Ok(transform) = std::env::var(transform) {
            let transform = transform
                .split(' ')
                .map(|x| x.parse::<f32>().unwrap())
                .collect::<Vec<f32>>();

            let mut pose = chain.origin();

            if transform.len() == 7 {
                let quaternion =
                    Quaternion::new(transform[3], transform[4], transform[5], transform[6])
                        .normalize(); // Example quaternion
                let rot = UnitQuaternion::from_quaternion(quaternion);
                pose.append_rotation_mut(&rot);
            }
            pose.append_translation_mut(&Translation3::new(
                transform[0],
                transform[1],
                transform[2],
            ));
            chain.set_origin(pose);
            chains.insert(path, chain);
        } else {
            // If no transform is set, use the default origin
            chains.insert(path, chain);
        }
    }

    Ok(chains)
}

pub fn update_visualization(
    rec: &RecordingStream,
    chain: &Chain<f32>,
    id: &str,
    positions: &[f32],
) -> Result<()> {
    chain.set_joint_positions_clamped(positions);

    chain.update_transforms();
    chain.update_link_transforms();

    for link_name in chain.iter_links().map(|link| link.name.clone()) {
        let link = chain.find_link(&link_name).context("Could not find link")?;
        let entity_path = get_entity_path(link, id);
        let link_to_world = link
            .world_transform()
            .context("Could not get world transform")?;
        let link_to_parent = if let Some(parent) = link.parent() {
            parent
                .world_transform()
                .context("Could not get world transform")?
                .inv_mul(&link_to_world)
        } else {
            link_to_world
        };
        let link_to_parent_mat = link_to_parent.to_matrix();

        let trans = link_to_parent_mat.column(3);
        let trans = trans.as_slice();
        let quat = link_to_parent.rotation.quaternion();
        let point_transform = rerun::Transform3D::from_translation_rotation(
            Vec3D::new(trans[0], trans[1], trans[2]),
            rerun::Quaternion::from([quat.i, quat.j, quat.k, quat.w]),
        );
        rec.log(entity_path.clone(), &point_transform)
            .context("Could not log transform")?;
    }

    Ok(())
}
