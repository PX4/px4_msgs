#!/bin/bash
set -euo pipefail

# Regenerate CHANGELOG.rst from the git tag history in the catkin_pkg / bloom
# RST format. Each released version entry is derived mechanically:
#   * the date comes from the version tag,
#   * the contributors come from the git authors in that tag's range,
#   * the summary comes from the msg/srv diff against the previous tag.
#
# The hand-curated "Forthcoming" section (unreleased changes) at the top of the
# existing CHANGELOG.rst is preserved verbatim.
#
# Usage:
#   scripts/generate_changelog.sh            # write the changelog to stdout
#   scripts/generate_changelog.sh --in-place # overwrite CHANGELOG.rst
#   scripts/generate_changelog.sh --check    # exit 1 if CHANGELOG.rst is stale

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
REPO_ROOT="$( realpath "${SCRIPT_DIR}/.." )"
cd "${REPO_ROOT}"

CHANGELOG="CHANGELOG.rst"
mode="stdout"
case "${1:-}" in
  --in-place) mode="in-place" ;;
  --check)    mode="check" ;;
  "")         mode="stdout" ;;
  *) echo "Unknown option: $1" >&2; exit 2 ;;
esac

underline() { # text char
  printf '%s\n' "$1"
  printf '%*s\n' "${#1}" '' | tr ' ' "$2"
}

join_csv() { paste -sd ',' - | sed 's/,/, /g'; }

names() { # base tag filter
  git diff --diff-filter="$3" --name-only "$1" "$2" -- msg srv \
    | sed 's#.*/##; s/\.\(msg\|srv\)$//' | sort | join_csv
}

contributors() { # range_or_empty
  local range="$1" out
  if [ -n "${range}" ]; then
    out="$(git log "${range}" --format='%an' 2>/dev/null \
      | sed -E 's/.*[Bb]uild ?[Bb]ot.*/PX4 BuildBot/' \
      | sort -u | grep -v '^$' | join_csv)"
  fi
  [ -n "${out:-}" ] && echo "${out}" || echo "PX4 BuildBot"
}

# Preserve an existing "Forthcoming" block (from its heading up to the first
# released "X.Y.Z (" heading), if present.
forthcoming_block() {
  if [ -f "${CHANGELOG}" ]; then
    awk '
      /^Forthcoming$/ {grab=1}
      grab && /^[0-9]+\.[0-9]+\.[0-9]+ \(/ {exit}
      grab {print}
    ' "${CHANGELOG}"
  fi
}

generate() {
  underline "Changelog for package px4_msgs" "^"
  echo
  cat <<'EOF'
The version numbers of this package track the corresponding `PX4 Autopilot
<https://github.com/PX4/PX4-Autopilot>`_ release line. The message and service
definitions are generated and synchronized automatically from the uORB
definitions in PX4-Autopilot, so the entries below are produced from the tagged
release history rather than by manual edits (see
``scripts/generate_changelog.sh``).
EOF
  echo

  # The "Forthcoming" section is the standard catkin/bloom staging area for
  # unreleased changes. It is optional here: it is kept only when it has been
  # curated with real content, otherwise it is omitted.
  local fc
  fc="$(forthcoming_block | sed -E '/^Forthcoming$/d; /^-+$/d; /^[[:space:]]*$/d; /^\*[[:space:]]*\(none\)[[:space:]]*$/d')"
  if [ -n "${fc}" ]; then
    printf '%s\n\n' "$(forthcoming_block)" | sed -E '/^[[:space:]]*$/{ N; /^[[:space:]]*\n[[:space:]]*$/D }'
  fi

  # Iterate tags newest -> oldest.
  local tags
  tags="$(git tag --sort=-v:refname | grep -E '^v[0-9]+\.[0-9]+\.[0-9]+$' || true)"
  while read -r tag; do
    [ -z "${tag}" ] && continue
    local version date prev_tag range
    version="${tag#v}"
    date="$(git log -1 --format=%ad --date=short "${tag}")"
    prev_tag="$(git tag --sort=v:refname | grep -E '^v[0-9]+\.[0-9]+\.[0-9]+$' \
      | awk -v t="${tag}" '$0==t{print p; exit} {p=$0}')"

    underline "${version} (${date})" "-"

    if [ -z "${prev_tag}" ]; then
      echo "* Initial tracked release, synchronized with PX4 Autopilot ${version}."
      range=""
    elif [ "$(git rev-parse "${prev_tag}^{tree}")" = "$(git rev-parse "${tag}^{tree}")" ]; then
      echo "* PX4 point release; message and service definitions identical to ${prev_tag#v}."
      range=""
    else
      echo "* Synchronized with PX4 Autopilot ${version}."
      local added removed changed
      added="$(names "${prev_tag}" "${tag}" A)"
      removed="$(names "${prev_tag}" "${tag}" D)"
      changed="$(git diff --diff-filter=M --name-only "${prev_tag}" "${tag}" -- msg srv | wc -l | tr -d ' ')"
      [ -n "${added}" ]   && echo "* Added: ${added}."
      [ -n "${removed}" ] && echo "* Removed: ${removed}."
      [ "${changed}" != "0" ] && echo "* Changed ${changed} message/service definition(s)."
      range="${prev_tag}..${tag}"
    fi
    echo "* Contributors: $(contributors "${range}")"
    echo
  done <<< "${tags}"
}

# Emit the changelog with exactly one trailing newline (no trailing blank line).
emit() { local c; c="$(generate)"; printf '%s\n' "${c}"; }

case "${mode}" in
  stdout)   emit ;;
  in-place) emit > "${CHANGELOG}"; echo "Wrote ${CHANGELOG}" >&2 ;;
  check)
    tmp="$(mktemp)"
    emit > "${tmp}"
    if ! diff -q "${tmp}" "${CHANGELOG}" >/dev/null 2>&1; then
      echo "CHANGELOG.rst is out of date. Run: scripts/generate_changelog.sh --in-place" >&2
      diff -u "${CHANGELOG}" "${tmp}" || true
      rm -f "${tmp}"; exit 1
    fi
    rm -f "${tmp}"; echo "CHANGELOG.rst is up to date." >&2 ;;
esac
