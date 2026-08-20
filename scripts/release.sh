#!/usr/bin/env bash
# QRNix firmware release script.
#
# Human steps before running:
#   1. Bump SOFTWARE_VERSION in src/qrnix.cpp to match the tag you will create.
#   2. Regenerate CHANGELOG.md (git-cliff -o CHANGELOG.md) and commit both.
#
# Usage:
#   scripts/release.sh v0.3.6            # full release
#   scripts/release.sh v0.3.6 --dry-run  # everything except tag, push, publish
set -euo pipefail

cd "$(dirname "$0")/.."

ENV_NAME="teensy40"
BUILD_DIR=".pio/build/${ENV_NAME}"
ELF="${BUILD_DIR}/firmware.elf"
HEX="${BUILD_DIR}/firmware.hex"

TAG="${1:-}"
MODE="${2:-}"

if [[ -z "$TAG" ]]; then
  echo "usage: $0 vX.Y.Z [--dry-run]" >&2
  exit 1
fi
if [[ ! "$TAG" =~ ^v[0-9]+\.[0-9]+\.[0-9]+$ ]]; then
  echo "error: tag must look like v0.3.6, got '$TAG'" >&2
  exit 1
fi
if [[ -n "$MODE" && "$MODE" != "--dry-run" ]]; then
  echo "error: unknown argument '$MODE'" >&2
  exit 1
fi

VERSION="${TAG#v}"
DIST_DIR="dist"
# Asset name matches the tag: qrnix-teensy40-v0.3.7.hex (convention from v0.3.6).
HEX_NAME="${DIST_DIR}/qrnix-${ENV_NAME}-${TAG}.hex"
DRY_RUN=0
[[ "$MODE" == "--dry-run" ]] && DRY_RUN=1

# ── prerequisites ─────────────────────────────────────────────────────────
for tool in pio gh strings sha256sum; do
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

# ── release notes from the commit log ─────────────────────────────────────
# Changelog anchor: the newest tag other than the one being released.
PREV="$(git tag --sort=-version:refname | grep -vx "$TAG" | head -1 || true)"
if [[ -z "$PREV" ]]; then
  PREV="$(git describe --tags --abbrev=0 "$(git rev-parse HEAD)" 2>/dev/null || true)"
fi
NOTES="$(mktemp)"
trap 'rm -f "$NOTES"' EXIT
if command -v git-cliff >/dev/null; then
  if ((TAG_EXISTS)); then
    git-cliff "${PREV}..${TAG}" >"$NOTES"
  else
    # --tag names the section before the tag exists; without it the notes
    # would ship with a literal "## [unreleased]" header on the release page.
    git-cliff --unreleased --tag "$TAG" >"$NOTES"
  fi
else
  if ((TAG_EXISTS)); then
    git log --oneline --no-merges "${PREV}..${TAG}" >"$NOTES"
  else
    git log --oneline --no-merges ${PREV:+${PREV}..HEAD} >"$NOTES"
  fi
fi
echo "== release notes preview =="
cat "$NOTES"

if ((DRY_RUN)); then
  echo "== DRY RUN: stopping before tag creation, push, and publish =="
fi

# ── tag ───────────────────────────────────────────────────────────────────
if ((!DRY_RUN && !TAG_EXISTS)); then
  git tag -a "$TAG" -m "QRNix Firmware $TAG"
fi

# ── build ─────────────────────────────────────────────────────────────────
pio run -e "$ENV_NAME"

# ── assert: compiled version string must equal the tag ────────────────────
# NOTE: no -q on grep: grep -q closes the pipe on first match, strings gets
# SIGPIPE, and pipefail turns that into a spurious failure.
if ! strings "$ELF" | grep -Fx "$VERSION"; then
  echo "error: $ELF does not contain version '$VERSION' (check SOFTWARE_VERSION in src/qrnix.cpp)" >&2
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
  echo "== DRY RUN: artifacts written to $DIST_DIR/, nothing published =="
  exit 0
fi

# ── publish ───────────────────────────────────────────────────────────────
# owner/repo from the remote URL. Strip .git FIRST: the greedy capture would
# otherwise swallow it and the API path would 404.
REPO="$(git remote get-url origin | sed -E 's#\.git$##' | sed -E 's#.*[:/]([^/]+/[^/]+)$#\1#')"
git push origin main
git push origin "$TAG"
gh release create "$TAG" "$HEX_NAME" \
  --title "QRNix Firmware $TAG" \
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
