#!/bin/bash
set -euo pipefail

# Generate Markdown release notes for a px4_msgs version by diffing the message
# and service definitions against the previous release. The notes mirror the
# upstream PX4 uORB changes: which interfaces were added, removed, or changed
# (with field/constant level detail).
#
# Usage:
#   scripts/generate_release_notes.sh [<tag>] [<base-ref>]
#
#   <tag>       Release tag to describe (default: the version from package.xml,
#               prefixed with "v").
#   <base-ref>  Ref to diff against (default: the previous version tag).
#
# The output is written to stdout.

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
REPO_ROOT="$( realpath "${SCRIPT_DIR}/.." )"
cd "${REPO_ROOT}"

# Resolve the tag to describe.
tag="${1:-}"
if [ -z "${tag}" ]; then
  version="$(sed -n 's:.*<version>\(.*\)</version>.*:\1:p' package.xml | head -n1)"
  tag="v${version}"
fi

# Resolve the base ref (previous version tag) unless given explicitly.
base="${2:-}"
if [ -z "${base}" ]; then
  base="$(git tag --sort=v:refname | awk -v t="${tag}" '$0==t{print p; exit} {p=$0}')"
fi

pxver="${tag#v}"

header() {
  echo "Message and service definitions synchronized with **PX4 Autopilot ${pxver}**."
  echo
  echo "Changes are mirrored from the upstream uORB definitions in [PX4-Autopilot](https://github.com/PX4/PX4-Autopilot)."
}

# No previous release to diff against: describe the baseline.
if [ -z "${base}" ]; then
  header
  echo
  count="$(git ls-tree -r --name-only "${tag}" -- msg srv | grep -cE '\.(msg|srv)$' || true)"
  echo "Initial release of this line, providing ${count} message and service definitions."
  exit 0
fi

# Re-tag of the same commit: no interface changes.
if [ "$(git rev-parse "${base}^{tree}")" = "$(git rev-parse "${tag}^{tree}")" ]; then
  header
  echo
  echo "This PX4 point release does not change any of the tracked message or service definitions — the interfaces are **identical to [${base}](https://github.com/PX4/px4_msgs/releases/tag/${base})**."
  exit 0
fi

# Extract the set of declarations (fields/constants) of a file at a ref, with
# comments and whitespace normalized away so comment-only churn is ignored.
decls() { # ref path
  git show "$1:$2" 2>/dev/null \
    | sed -E 's/#.*$//' \
    | sed -E 's/[[:space:]]+/ /g; s/^ //; s/ $//' \
    | grep -vE '^$' \
    | sort -u
}

namelist() { # filter
  git diff --diff-filter="$1" --name-only "${base}" "${tag}" -- msg srv \
    | sed 's#^msg/##; s#^srv/##' | sort
}

header
echo
echo "Summary of interface changes since [${base}](https://github.com/PX4/px4_msgs/releases/tag/${base}):"
echo

added="$(namelist A)"
removed="$(namelist D)"
modified="$(git diff --diff-filter=M --name-only "${base}" "${tag}" -- msg srv | sort)"

if [ -n "${added}" ]; then
  echo "### Added"
  echo
  while read -r f; do [ -n "$f" ] && echo "- \`$f\`"; done <<< "${added}"
  echo
fi

if [ -n "${removed}" ]; then
  echo "### Removed"
  echo
  while read -r f; do [ -n "$f" ] && echo "- \`$f\`"; done <<< "${removed}"
  echo
fi

if [ -n "${modified}" ]; then
  echo "### Changed"
  echo
  while read -r f; do
    [ -z "$f" ] && continue
    tmpb="$(mktemp)"; tmpt="$(mktemp)"
    decls "${base}" "$f" > "${tmpb}"
    decls "${tag}" "$f" > "${tmpt}"
    add_d="$(comm -13 "${tmpb}" "${tmpt}")"
    rem_d="$(comm -23 "${tmpb}" "${tmpt}")"
    rm -f "${tmpb}" "${tmpt}"
    if [ -z "${add_d}" ] && [ -z "${rem_d}" ]; then
      echo "- **\`$(basename "$f")\`** — documentation/comment updates only"
      continue
    fi
    echo "- **\`$(basename "$f")\`**"
    while read -r l; do [ -n "$l" ] && echo "  - added \`$l\`"; done <<< "${add_d}"
    while read -r l; do [ -n "$l" ] && echo "  - removed \`$l\`"; done <<< "${rem_d}"
  done <<< "${modified}"
  echo
fi
