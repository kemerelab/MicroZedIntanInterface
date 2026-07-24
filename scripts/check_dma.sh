#!/usr/bin/env bash
# SPDX-License-Identifier: MIT
# SPDX-FileCopyrightText: 2025-2026 Caleb Kemere, Reet Sinha, Allen Mikhailov, Rice University

# check_dma.sh -- guardrail for the PL->PS data path.
#
# RULE: bulk PL->PS data must move by DMA (AXI CDMA), landed straight into the
# pbuf payload; the PS must NOT loop over BRAM or the DMA staging buffer on the
# CPU. This script flags the two single-beat patterns:
#   A) Xil_In32/Xil_Out32 of a *BRAM* address          (single-beat BRAM access)
#   B) a CPU read of the DMA staging buffer: "= ...staging...[...]"
# Isolated control/status-register accesses (PL_CTRL_BASE_ADDR + STATUS/CTRL_REG)
# are NOT flagged -- those are small config, not bulk data.
#
# To keep a genuinely-justified single-beat site (e.g. a 2-word magic/resync
# peek), annotate the SAME line with:   // DMA-EXEMPT: <reason>
#
# Usage:  scripts/check_dma.sh [dir]      (default: firmware)
# Exit 0 = clean (all sites DMA or exempt); 1 = un-justified single-beat sites.
# Run this before declaring any PL<->PS data-path task done.
set -uo pipefail
DIR="${1:-firmware}"
SRC=$(find "$DIR" \( -name '*.c' -o -name '*.h' \) 2>/dev/null)
[ -z "$SRC" ] && { echo "check_dma: no C sources under '$DIR'"; exit 0; }

viol=0; exem=0
report() {  # $1 = label, $2 = ERE
  local label="$1" pat="$2" hit
  while IFS= read -r hit; do
    [ -z "$hit" ] && continue
    case "$hit" in
      *DMA-EXEMPT*) printf '  exempt   %-17s %s\n' "$label" "${hit%%:*}:$(printf '%s' "$hit" | cut -d: -f2)"; exem=$((exem+1)) ;;
      *) printf 'VIOLATION  %-17s %s\n             %s\n' "$label" \
              "${hit%%:*}:$(printf '%s' "$hit" | cut -d: -f2)" \
              "$(printf '%s' "$hit" | cut -d: -f3- | sed 's/^[[:space:]]*//')"; viol=$((viol+1)) ;;
    esac
  done < <(grep -HnE "$pat" $SRC 2>/dev/null)
}

report "single-beat-BRAM" 'Xil_(In|Out)32[[:space:]]*\([^;]*BRAM'
report "staging-CPU-read" '=[[:space:]]*[A-Za-z_]*staging[A-Za-z0-9_]*\['

echo "------------------------------------------------------------"
if [ "$viol" -gt 0 ]; then
  echo "check_dma: FAIL -- $viol un-justified single-beat site(s); $exem exempt."
  echo "  Move the data by DMA straight into the pbuf payload (PS writes only the header,"
  echo "  or have the PL build the whole packet in BRAM and just DMA+send it)."
  echo "  If a site is genuinely justified, annotate the line: // DMA-EXEMPT: <reason>"
  exit 1
fi
echo "check_dma: PASS -- no un-justified single-beat transfers ($exem exempt)."
