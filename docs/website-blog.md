# Publishing blog posts on the website

The website in [dora-rs/dora-rs.github.io](https://github.com/dora-rs/dora-rs.github.io)
imports Markdown from this repository's `docs/blog` folder at build time. Keep
posts here; there is no separate website copy to maintain.

Use a filename like `YYYY-MM-DD-post-title.md`, an H1 title, and an introductory
paragraph. The filename provides the publication date and stable URL at
`https://dora-rs.ai/blog/YYYY-MM-DD-post-title/`. No frontmatter is required.
Keep drafts outside `docs/blog`: all dated posts on `main` are published.

When a change under `docs/blog/**` reaches `main`, the
[Rebuild website blog workflow](../.github/workflows/website-blog.yml) immediately
requests the website's `deploy.yml` workflow. The website checks out the latest
posts from `main`, builds, and deploys to GitHub Pages. Publication takes the
normal build and deployment time, rather than waiting for a scheduled rebuild.
Edits, renames and deletions also trigger a rebuild.

## One-time setup

1. Merge the blog integration and `workflow_dispatch` trigger into the website
   repository's `.github/workflows/deploy.yml` first.
2. Create a fine-grained personal access token with resource owner **dora-rs**,
   repository access limited to **dora-rs.github.io**, and repository permission
   **Actions: Read and write**. Complete organization approval if required.
3. In **dora-rs/dora → Settings → Secrets and variables → Actions**, create a
   repository secret named **WEBSITE_DISPATCH_TOKEN** with that token. An
   organization secret restricted to `dora` also works. Do not commit the token.
4. Run **Actions → Rebuild website blog → Run workflow** on `main`, and check
   **dora-rs/dora-rs.github.io → Actions → Deploy to GitHub Pages** for the build.

The standard `GITHUB_TOKEN` cannot access the other repository. Dispatching a
workflow accepts a fine-grained token with Actions write permission; see
[GitHub's workflow dispatch API](https://docs.github.com/en/rest/actions/workflows#create-a-workflow-dispatch-event).
Rotate the secret before its token expires. A missing or expired secret fails
the dispatch job visibly. The website can also be rebuilt manually through its
own **Deploy to GitHub Pages** workflow.
