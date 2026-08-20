#!/usr/bin/env bash
# QRNix firmware release driver.
#
# One command does the whole release: pre-flight checks, version bump in
# src/qrnix.cpp, changelog regeneration, notes, tag, build, version assert,
# packaging, push, GitHub release, digest verification.
#
# Usage:
#   scripts/release.sh                 # next patch version from latest tag
#   scripts/release.sh v0.3.10         # explicit version
#   scripts/release.sh v0.3.10 -y      # skip the confirmation prompt
#   scripts/release.sh v0.3.10 --dry-run  # rehearsal: no tag, no push, no publish
set -euo pipefail

cd "$(dirname "$0")/.."

ENV_NAME="teensy40"
BUILD_DIR=".pio/build/${ENV_NAME}"
ELF="${BUILD_DIR}/firmware.elf"
HEX="${BUILD_DIR}/firmware.hex"
DIST_DIR="dist"
SRC="src/qrnix.cpp"

# Mode flag may come first (release.sh --dry-run) or after the version.
if [[ "${1:-}" == "--dry-run" || "${1:-}" == "-y" ]]; then
  MODE="${1}"
  TAG="${2:-}"
else
  TAG="${1:-}"
  MODE="${2:-}"
fi
DRY_RUN=0
YES=0

if [[ -n "$MODE" && "$MODE" != "--dry-run" && "$MODE" != "-y" ]]; then
  echo "error: unknown argument '$MODE' (use --dry-run or -y)" >&2
  exit 1
fi
[[ "$MODE" == "--dry-run" ]] && DRY_RUN=1
[[ "$MODE" == "-y" ]] && YES=1

# ── version: explicit arg, or next patch from the latest tag ──────────────
if [[ -z "$TAG" ]]; then
  LATEST="$(git tag --sort=-version:refname | head -1 || true)"
  if [[ -z "$LATEST" ]]; then
    echo "error: no tags found; pass the version explicitly" >&2
    exit 1
  fi
  IFS=. read -r maj min pat <<< "${LATEST#v}"
  TAG="v${maj}.${min}.$((pat + 1))"
  echo "no version given, using next patch: $TAG (from $LATEST)"
fi
if [[ ! "$TAG" =~ ^v[0-9]+\.[0-9]+\.[0-9]+$ ]]; then
  echo "error: tag must look like v0.3.10, got '$TAG'" >&2
  exit 1
fi

VERSION="${TAG#v}"
# Asset name matches the tag: qrnix-teensy40-v0.3.10.hex.
HEX_NAME="${DIST_DIR}/qrnix-${ENV_NAME}-${TAG}.hex"

# ── pre-flight ────────────────────────────────────────────────────────────
for tool in pio gh strings sha256sum git-cliff; do
  command -v "$tool" >/dev/null || { echo "error: $tool not found" >&2; exit 1; }
done
[[ "$(git branch --show-current)" == "main" ]] || { echo "error: releases happen on main" >&2; exit 1; }
if ! git diff --quiet || ! git diff --cached --quiet; then
  echo "error: uncommitted tracked changes; commit or stash first" >&2
  exit 1
fi

TAG_EXISTS=0
if git rev-parse -q --verify "refs/tags/$TAG" >/dev/null; then
  if [[ "$(git rev-parse "$TAG^{}")" != "$(git rev-parse HEAD)" ]]; then
    echo "error: tag $TAG exists but does not point at HEAD; refusing" >&2
    exit 1
  fi
  TAG_EXISTS=1
fi

# Changelog anchor: the newest tag other than the one being released.
PREV="$(git tag --sort=-version:refname | grep -vx "$TAG" | head -1 || true)"
if [[ -z "$PREV" ]]; then
  PREV="$(git describe --tags --abbrev=0 "$(git rev-parse HEAD)" 2>/dev/null || true)"
fi

# ── prep: bump version, regenerate changelog, commit (idempotent) ─────────
PREP_DONE=0
if [[ "$(git log -1 --format=%s)" == "Prepare release $TAG" ]]; then
  if grep -q "SOFTWARE_VERSION = \"$VERSION\"" "$SRC"; then
    PREP_DONE=1
    echo "prep for $TAG already done, reusing"
  else
    echo "error: last commit is a prep for $TAG but $SRC does not contain version $VERSION; fix manually" >&2
    exit 1
  fi
fi

if ((!PREP_DONE)); then
  # Clean-start requirement: main must be in sync with origin before the
  # driver creates the prep commit.
  git fetch origin
  if ! git rev-parse -q --verify origin/main >/dev/null; then
    echo "error: origin/main does not exist; push main first" >&2
    exit 1
  fi
  if [[ "$(git rev-parse HEAD)" != "$(git rev-parse origin/main)" ]]; then
    echo "error: main is not in sync with origin/main; push or pull first" >&2
    exit 1
  fi
  if [[ -z "$PREV" ]]; then
    COUNT="$(git rev-list --count HEAD)"
  else
    COUNT="$(git rev-list --count "${PREV}..HEAD")"
  fi
  if ((COUNT == 0)); then
    echo "error: nothing to release (no commits since ${PREV:-the beginning})" >&2
    exit 1
  fi

  # Version bump in the source; the build assert re-verifies it later.
  sed -i "s/SOFTWARE_VERSION = \"[^\"]*\"/SOFTWARE_VERSION = \"$VERSION\"/" "$SRC"
  grep -q "SOFTWARE_VERSION = \"$VERSION\"" "$SRC" || { echo "error: version bump failed in $SRC" >&2; exit 1; }
  git-cliff -o CHANGELOG.md
  git add "$SRC" CHANGELOG.md
  git commit -q -m "Prepare release $TAG"
  echo "prep committed: Prepare release $TAG"
fi

# ── release notes from the commit log ─────────────────────────────────────
NOTES="$(mktemp)"
trap 'rm -f "$NOTES"' EXIT
if ((TAG_EXISTS)); then
  git-cliff "${PREV}..${TAG}" >"$NOTES"
else
  # --tag names the section before the tag exists; without it the notes
  # would ship with a literal "## [unreleased]" header on the release page.
  git-cliff --unreleased --tag "$TAG" >"$NOTES"
fi
echo "== release notes preview =="
cat "$NOTES"
if ! [[ -s "$NOTES" ]]; then
  echo "warning: notes are empty (no conventional commits since ${PREV:-the beginning})" >&2
fi

# ── confirmation ──────────────────────────────────────────────────────────
if ((!DRY_RUN)); then
  if ((!YES)); then
    echo "== release summary =="
    echo "version:  $TAG"
    echo "tag at:   $(git rev-parse --short HEAD) ($(git log -1 --format=%s))"
    echo "actions:  create tag, build, push main, push tag, create GitHub release, verify digest"
    read -r -p "Proceed? [y/N] " ans < /dev/tty || ans="n"
    if [[ "$ans" != "y" && "$ans" != "Y" ]]; then
      echo "aborted" >&2
      exit 1
    fi
  fi

  # ── tag ─────────────────────────────────────────────────────────────────
  if ((!TAG_EXISTS)); then
    git tag -a "$TAG" -m "QRNix $TAG"
  fi
fi

# ── build ─────────────────────────────────────────────────────────────────
pio run -e "$ENV_NAME"

# ── assert: compiled version string must equal the tag ────────────────────
# NOTE: no -q on grep: grep -q closes the pipe on first match, strings gets
# SIGPIPE, and pipefail turns that into a spurious failure.
if ! strings "$ELF" | grep -Fx "$VERSION"; then
  echo "error: $ELF does not contain version '$VERSION' (SOFTWARE_VERSION in $SRC)" >&2
  exit 1
fi
[[ -f "$HEX" ]] || { echo "error: $HEX missing after build" >&2; exit 1; }

# ── package ───────────────────────────────────────────────────────────────
mkdir -p "$DIST_DIR"
cp "$HEX" "$HEX_NAME"
LOCAL_SHA="$(sha256sum "$HEX_NAME" | cut -d' ' -f1)"
echo "== artifacts =="
echo "$LOCAL_SHA  $HEX_NAME"

if ((DRY_RUN)); then
  echo "== DRY RUN: artifacts written to $DIST_DIR/, nothing tagged or published =="
  exit 0
fi

# ── publish ───────────────────────────────────────────────────────────────
# owner/repo from the remote URL. Strip .git FIRST: the greedy capture would
# otherwise swallow it and the API path would 404.
REPO="$(git remote get-url origin | sed -E 's#\.git$##' | sed -E 's#.*[:/]([^/]+/[^/]+)$#\1#')"
git push origin main
git push origin "$TAG"
gh release create "$TAG" "$HEX_NAME" \
  --title "QRNix $TAG" \
  --notes-file "$NOTES"

# GitHub computes the asset digest from the uploaded bytes; it must equal the
# hash of the hex built from the tagged commit. A mismatch means the wrong
# file was uploaded. The digest can lag the upload and the API can fail, so
# retry until the value looks like a real sha256 (64 hex chars); anything
# else (error JSON, empty) is a failed read, not a digest.
REMOTE_SHA=""
for _ in 1 2 3 4 5; do
  REMOTE_SHA="$(gh api "repos/${REPO}/releases/tags/${TAG}" \
    --jq ".assets[] | select(.name == \"$(basename "$HEX_NAME")\") | .digest" \
    | tr -d '"' | sed 's/^sha256://')" || true
  [[ "$REMOTE_SHA" =~ ^[0-9a-f]{64}$ ]] && break
  REMOTE_SHA=""
  sleep 2
done
if [[ -z "$REMOTE_SHA" ]]; then
  echo "warning: release created but the uploaded digest could not be read; compare manually on the release page" >&2
  exit 0
fi
if [[ "$REMOTE_SHA" != "$LOCAL_SHA" ]]; then
  echo "error: uploaded digest ${REMOTE_SHA} != local build hash ${LOCAL_SHA}; delete the release and investigate" >&2
  exit 1
fi
echo "verified: GitHub digest ${REMOTE_SHA} matches the local build"
echo "released $TAG: $HEX_NAME"
