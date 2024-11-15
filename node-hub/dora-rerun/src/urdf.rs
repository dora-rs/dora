use std::collections::HashMap;

use eyre::{Context, ContextCompat, Result};
use k::Chain;
use rerun::{components::RotationAxisAngle, Angle, RecordingStream, Rotation3D, Vec3D};
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
        .filter(|(key, _)| key.ends_with("_urdf"))
        .collect::<Vec<_>>();
    let mut chains = HashMap::new();
    for (key, urdf_path) in urdfs {
        let path = key.replace("_urdf", ".urdf");
        let chain = k::Chain::<f32>::from_urdf_file(&urdf_path).context("Could not load URDF")?;

        let transform = key.replace("_urdf", "_transform");
        if let Err(err) = rec.log_file_from_path(&urdf_path, None, false) {
            println!("Could not log file: {}. Errored with {}", urdf_path, err);
            println!("Make sure to install urdf loader with:");
            println!(
                "pip install git+https://github.com/rerun-io/rerun-loader-python-example-urdf.git"
            )
        };
        // Get transform by replacing URDF_ with TRANSFORM_
        if let Ok(transform) = std::env::var(transform) {
            let transform = transform
                .split(' ')
                .map(|x| x.parse::<f32>().unwrap())
                .collect::<Vec<f32>>();
            rec.log(
                path.clone(),
                &rerun::Transform3D::from_translation_rotation(
                    [transform[0], transform[1], transform[2]],
                    Rotation3D::AxisAngle(RotationAxisAngle::new(
                        [0., 0., 0.],
                        Angle::from_degrees(0.0),
                    )),
                ),
            )
            .unwrap();
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
        let link_to_parent = if link_name != "base_link" {
            let parent = link.parent().context("could not get parent")?;
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
        rec.log(
            entity_path,
            &rerun::Transform3D::from_translation_rotation(
                Vec3D::new(trans[0], trans[1], trans[2]),
                rerun::Quaternion::from([quat.i, quat.j, quat.k, quat.w]),
            ),
        )
        .context("Could not log transform")?;
    }
    Ok(())
}
