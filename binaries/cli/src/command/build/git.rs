use dora_message::{common::GitSource, descriptor::GitRepoRev};
use eyre::Context;

pub fn fetch_commit_hash(repo_url: String, rev: Option<GitRepoRev>) -> eyre::Result<GitSource> {
    let mut remote = git2::Remote::create_detached(repo_url.as_bytes())
        .with_context(|| format!("failed to create git remote for {repo_url}"))?;
    let connection = remote
        .connect_auth(git2::Direction::Fetch, None, None)
        .with_context(|| format!("failed to open connection to {repo_url}"))?;
    let references = connection
        .list()
        .with_context(|| format!("failed to list git references of {repo_url}"))?;

    let expected_name = match &rev {
        Some(GitRepoRev::Branch(branch)) => format!("refs/heads/{branch}"),
        Some(GitRepoRev::Tag(tag)) => format!("refs/tags/{tag}"),
        Some(GitRepoRev::Rev(rev)) => rev.clone(),
        None => "HEAD".into(),
    };

    let mut commit_hash = None;
    for head in references {
        if head.name() == expected_name {
            commit_hash = Some(head.oid().to_string());
            break;
        }
    }

    if commit_hash.is_none()
        && let Some(GitRepoRev::Rev(rev)) = &rev
        && looks_like_commit_hash(rev)
    {
        // rev wasn't found among the references, but it looks like a raw
        // commit hash, so try to use it directly.
        commit_hash = Some(rev.clone());
    }

    match commit_hash {
        Some(commit_hash) => Ok(GitSource {
            repo: repo_url,
            commit_hash,
            subdir: None,
            hub: None,
        }),
        None => eyre::bail!("no matching commit for `{rev:?}`"),
    }
}

/// Returns whether `rev` could be a git object id (a short or full commit
/// hash).
///
/// Git object ids are hexadecimal, between 4 (the minimum abbreviation git
/// accepts) and 40 (a full SHA-1) characters long. Restricting to this range
/// rejects mistyped refs like `mybranchtypo` early with the clear "no matching
/// commit" error instead of letting them fail later with a lower-level git
/// error.
fn looks_like_commit_hash(rev: &str) -> bool {
    (4..=40).contains(&rev.len()) && rev.bytes().all(|b| b.is_ascii_hexdigit())
}

#[cfg(test)]
mod tests {
    use super::looks_like_commit_hash;

    #[test]
    fn accepts_short_and_full_hashes() {
        assert!(looks_like_commit_hash("abcd"));
        assert!(looks_like_commit_hash("a7b8969"));
        assert!(looks_like_commit_hash(
            "a7b89691e0000000000000000000000000000000"
        ));
        assert!(looks_like_commit_hash("0123456789ABCDEF"));
    }

    #[test]
    fn rejects_non_hex_revs() {
        assert!(!looks_like_commit_hash("mybranchtypo"));
        assert!(!looks_like_commit_hash("zzzzzzz"));
        assert!(!looks_like_commit_hash("main"));
    }

    #[test]
    fn rejects_out_of_range_lengths() {
        assert!(!looks_like_commit_hash(""));
        assert!(!looks_like_commit_hash("abc"));
        // longer than a full SHA-1
        assert!(!looks_like_commit_hash(
            "a7b89691e00000000000000000000000000000000"
        ));
    }
}
