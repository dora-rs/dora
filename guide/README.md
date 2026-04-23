# Dora User Guide

Built with [mdBook](https://rust-lang.github.io/mdBook/). Available in English and Chinese (zh-CN).

## Prerequisites

```bash
cargo install mdbook
cargo install mdbook-i18n-helpers
```

## Build and Serve

```bash
# English (opens browser with live-reload)
mdbook serve --open

# Chinese
MDBOOK_BOOK__LANGUAGE=zh-CN mdbook serve -d book/zh-CN --open
```

## Build Only

```bash
# English
mdbook build

# Chinese
MDBOOK_BOOK__LANGUAGE=zh-CN mdbook build -d book/zh-CN
```

## Updating Translations

After editing English source files in `src/`, re-extract and update translations:

```bash
# Re-extract translatable strings
MDBOOK_OUTPUT='{"xgettext": {"pot-file": "messages.pot"}}' mdbook build -d po

# Apply translations to zh-CN.po
python3 translate.py

# Verify Chinese build
MDBOOK_BOOK__LANGUAGE=zh-CN mdbook build -d book/zh-CN
```
