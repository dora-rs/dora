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

    if commit_hash.is_none() {
        if let Some(GitRepoRev::Rev(rev)) = &rev {
            // rev might be a commit hash instead of a reference
            if is_commit_hash_candidate(rev) {
                commit_hash = Some(rev.clone());
            }
        }
    }

    match commit_hash {
        Some(commit_hash) => Ok(GitSource {
            repo: repo_url,
            commit_hash,
        }),
        None => eyre::bail!("no matching commit for `{rev:?}`"),
    }
}

fn is_commit_hash_candidate(rev: &str) -> bool {
    !rev.is_empty() && rev.is_ascii() && rev.bytes().all(|b| b.is_ascii_hexdigit())
}

#[cfg(test)]
mod tests {
    use super::is_commit_hash_candidate;

    #[test]
    fn accepts_hex_commit_hash_candidates() {
        assert!(is_commit_hash_candidate("deadbeef"));
        assert!(is_commit_hash_candidate("0123456789abcdef"));
        assert!(is_commit_hash_candidate("ABCDEF1234"));
    }

    #[test]
    fn rejects_non_hex_commit_hash_candidates() {
        assert!(!is_commit_hash_candidate(""));
        assert!(!is_commit_hash_candidate("release1"));
        assert!(!is_commit_hash_candidate("abcdefg"));
        assert!(!is_commit_hash_candidate("abc-def"));
    }
}
