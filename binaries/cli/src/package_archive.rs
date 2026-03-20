use crate::publish_metadata::{PublishManifest, PublishedPackageRecord};
use eyre::{Context, bail};
use flate2::{Compression, read::GzDecoder, write::GzEncoder};
use sha2::{Digest, Sha256};
use std::{
    io::{Cursor, Read},
    path::Path,
};
use tar::{Archive, Builder};

pub fn build_published_package(
    package_dir: &Path,
) -> eyre::Result<(PublishedPackageRecord, Vec<u8>)> {
    let manifest_path = package_dir.join("Dora.toml");
    let manifest = PublishManifest::from_dora_toml_path(&manifest_path)?;
    let archive = create_package_archive(package_dir)?;
    let checksum = archive_checksum(&archive);
    Ok((
        PublishedPackageRecord::from_manifest(manifest, checksum),
        archive,
    ))
}

pub fn create_package_archive(package_dir: &Path) -> eyre::Result<Vec<u8>> {
    if !package_dir.is_dir() {
        bail!(
            "package directory `{}` does not exist",
            package_dir.display()
        );
    }

    let mut encoder = GzEncoder::new(Vec::new(), Compression::default());
    {
        let mut builder = Builder::new(&mut encoder);
        append_directory(&mut builder, package_dir, package_dir)?;
        builder.finish()?;
    }
    encoder.finish().map_err(Into::into)
}

pub fn archive_checksum(archive_bytes: &[u8]) -> String {
    let checksum = Sha256::digest(archive_bytes);
    format!("sha256:{}", hex::encode(checksum))
}

pub fn validate_published_archive(
    record: &PublishedPackageRecord,
    archive_bytes: &[u8],
) -> eyre::Result<()> {
    let actual_checksum = archive_checksum(archive_bytes);
    if actual_checksum != record.checksum {
        bail!(
            "archive checksum mismatch for `{}@{}`: expected `{}`, got `{}`",
            record.name,
            record.version,
            record.checksum,
            actual_checksum
        );
    }

    let manifest = read_manifest_from_archive(archive_bytes)?;
    if manifest.name != record.name {
        bail!(
            "archive manifest name `{}` does not match published metadata `{}`",
            manifest.name,
            record.name
        );
    }
    if manifest.version != record.version {
        bail!(
            "archive manifest version `{}` does not match published metadata `{}`",
            manifest.version,
            record.version
        );
    }
    if manifest.dependencies != record.dependencies {
        bail!(
            "archive manifest dependencies do not match published metadata for `{}@{}`",
            record.name,
            record.version
        );
    }

    Ok(())
}

fn append_directory(
    builder: &mut Builder<&mut GzEncoder<Vec<u8>>>,
    root: &Path,
    current: &Path,
) -> eyre::Result<()> {
    for entry in std::fs::read_dir(current)
        .with_context(|| format!("failed to read directory `{}`", current.display()))?
    {
        let entry = entry?;
        let path = entry.path();
        let relative = path.strip_prefix(root).unwrap();

        if entry.file_type()?.is_dir() {
            builder.append_dir(relative, &path)?;
            append_directory(builder, root, &path)?;
        } else if entry.file_type()?.is_file() {
            let mut file = std::fs::File::open(&path)
                .with_context(|| format!("failed to open file `{}`", path.display()))?;
            builder
                .append_file(relative, &mut file)
                .with_context(|| format!("failed to archive file `{}`", path.display()))?;
        }
    }

    Ok(())
}

fn read_manifest_from_archive(archive_bytes: &[u8]) -> eyre::Result<PublishManifest> {
    let gz = GzDecoder::new(Cursor::new(archive_bytes));
    let mut archive = Archive::new(gz);

    for entry in archive.entries()? {
        let mut entry = entry?;
        let path = entry.path()?;
        if is_dora_manifest_path(path.as_ref()) {
            let mut manifest = String::new();
            entry.read_to_string(&mut manifest)?;
            return PublishManifest::from_dora_toml_str(&manifest);
        }
    }

    bail!("archive does not contain `Dora.toml`")
}

fn is_dora_manifest_path(path: &Path) -> bool {
    path.file_name()
        .and_then(|name| name.to_str())
        .map(|name| name == "Dora.toml")
        .unwrap_or(false)
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::io::Write;
    use std::path::PathBuf;
    use tar::Header;
    use tempfile::tempdir;

    #[test]
    fn builds_record_and_validates_archive() {
        let package_dir = create_test_package_dir().unwrap();

        let (record, archive) = build_published_package(package_dir.path()).unwrap();

        assert_eq!(record.name, "camera_node");
        assert_eq!(record.version.to_string(), "0.1.0");
        validate_published_archive(&record, &archive).unwrap();
    }

    #[test]
    fn archive_contains_manifest() {
        let package_dir = create_test_package_dir().unwrap();
        let archive = create_package_archive(package_dir.path()).unwrap();
        let manifest = read_manifest_from_archive(&archive).unwrap();

        assert_eq!(manifest.name, "camera_node");
    }

    #[test]
    fn validation_fails_for_checksum_mismatch() {
        let package_dir = create_test_package_dir().unwrap();
        let (mut record, archive) = build_published_package(package_dir.path()).unwrap();
        record.checksum = "sha256:deadbeef".to_owned();

        let err = validate_published_archive(&record, &archive)
            .unwrap_err()
            .to_string();

        assert!(err.contains("checksum mismatch"));
    }

    #[test]
    fn validation_fails_without_manifest() {
        let bytes = {
            let mut encoder = GzEncoder::new(Vec::new(), Compression::default());
            {
                let mut builder = Builder::new(&mut encoder);
                let content = b"hello";
                let mut header = Header::new_gnu();
                header.set_size(content.len() as u64);
                header.set_mode(0o644);
                header.set_cksum();
                builder
                    .append_data(
                        &mut header,
                        PathBuf::from("README.md"),
                        Cursor::new(content),
                    )
                    .unwrap();
                builder.finish().unwrap();
            }
            encoder.finish().unwrap()
        };

        let err = read_manifest_from_archive(&bytes).unwrap_err().to_string();
        assert!(err.contains("does not contain `Dora.toml`"));
    }

    fn create_test_package_dir() -> eyre::Result<tempfile::TempDir> {
        let dir = tempdir()?;
        std::fs::create_dir_all(dir.path().join("camera_node"))?;
        std::fs::write(
            dir.path().join("Dora.toml"),
            r#"
[package]
name = "camera_node"
version = "0.1.0"

[dependencies]
yolo = { version = "^1.2.0" }
"#,
        )?;
        let mut source = std::fs::File::create(dir.path().join("camera_node").join("main.py"))?;
        source.write_all(b"print('hello')\n")?;
        Ok(dir)
    }
}
