#!/usr/bin/env bash
set -euo pipefail

project_root=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
cd "$project_root"

failed=0
tracked=$(git ls-files)

if printf '%s\n' "$tracked" | grep -E '(^|/)(build|install|log|data|results|bags|maps|snapshots|vendor_src)/|\.(bag|db3|mcap|engine|onnx|pem|key)$' >/dev/null; then
  echo 'FAIL: generated, model, credential, or recording path is tracked.' >&2
  failed=1
fi

if git grep -n -I -E '/home/[^/]+|BEGIN [A-Z ]*PRIVATE KEY|Bearer [A-Za-z0-9_.-]{12,}|(password|token)[[:space:]]*[:=][[:space:]]*[A-Za-z0-9_.-]{12,}' -- . \
  ':!tools/publication_audit.sh'; then
  echo 'FAIL: possible local path or embedded credential found.' >&2
  failed=1
fi

if git grep -n -I -E 'team_|\<[Tt][Ee][Aa][Mm]\>' -- . \
  ':!LICENSE' ':!tools/publication_audit.sh'; then
  echo 'FAIL: legacy collaboration branding found.' >&2
  failed=1
fi

if [[ $failed != 0 ]]; then
  exit 1
fi

echo 'PASS: no tracked generated data, models, private keys, absolute home paths, or obvious embedded credentials.'
