#!/usr/bin/env python3
"""
plot_position.py
----------------
Plots p[0], p[1], p[2] from folder/eskf/State.csv over a user-defined
time window, with FSM-state regions overlaid from folder/FsmTransition.csv.

Large-file strategy: State.csv is read in chunks; only rows inside the
[start_ts, end_ts] window are kept in memory.
"""

import os
import sys
import pandas as pd
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
from matplotlib.ticker import AutoMinorLocator


# ── helpers ──────────────────────────────────────────────────────────────────

def prompt_folder() -> str:
    while True:
        folder = input("Enter the data folder path: ").strip()
        if os.path.isdir(folder):
            return folder
        print(f"  ✗  '{folder}' is not a valid directory. Try again.")


def prompt_timestamp(label: str, reference: int | None = None) -> int:
    while True:
        raw = input(f"Enter {label} timestamp (µs): ").strip()
        try:
            val = int(raw)
            if reference is not None and val <= reference:
                print(f"  ✗  Must be greater than {reference}.")
                continue
            return val
        except ValueError:
            print("  ✗  Please enter an integer.")


def load_fsm(fsm_path: str) -> pd.DataFrame:
    """Load FSM transitions; keep only the string state columns."""
    df = pd.read_csv(fsm_path)
    # Columns: ts_us, prev_state, new_state  (strings appear before ints)
    # Guard: take the first occurrence of each name (string version).
    # If the file has duplicate column names pandas appends .1, .2 …
    # so 'prev_state' is string, 'prev_state.1' is int — drop the .1/.2 ones.
    df = df.loc[:, ~df.columns.str.contains(r'\.\d+$')]
    df = df[['ts_us', 'new_state']].copy()
    df['ts_us'] = df['ts_us'].astype(int)
    df.sort_values('ts_us', inplace=True)
    df.reset_index(drop=True, inplace=True)
    return df


def load_state_chunked(
    state_path: str,
    ts_start: int,
    ts_end: int,
    chunksize: int = 500_000,
) -> pd.DataFrame:
    """Read State.csv in chunks, keeping only rows in [ts_start, ts_end]."""
    chunks = []
    total_rows = 0
    kept_rows = 0
    print(f"\nReading '{state_path}' in chunks of {chunksize:,} rows …")
    for chunk in pd.read_csv(state_path, chunksize=chunksize):
        total_rows += len(chunk)
        mask = (chunk['ts_us'] >= ts_start) & (chunk['ts_us'] <= ts_end)
        filtered = chunk.loc[mask, ['ts_us', 'p[0]', 'p[1]', 'p[2]']].copy()
        if not filtered.empty:
            chunks.append(filtered)
            kept_rows += len(filtered)
        # Early exit: once we've passed ts_end we can stop
        if chunk['ts_us'].iloc[-1] > ts_end:
            break

    print(f"  Scanned {total_rows:,} rows, kept {kept_rows:,} rows in window.")
    if not chunks:
        print("  ✗  No data found in the specified time window.")
        sys.exit(1)
    df = pd.concat(chunks, ignore_index=True)
    df.sort_values('ts_us', inplace=True)
    return df


def build_fsm_regions(
    fsm_df: pd.DataFrame,
    ts_start: int,
    ts_end: int,
    initial_state: str = "INIT",
) -> list[dict]:
    """
    Return a list of {state, t_start, t_end} dicts covering [ts_start, ts_end].
    """
    # Determine which state is active at ts_start
    before = fsm_df[fsm_df['ts_us'] <= ts_start]
    if before.empty:
        current_state = initial_state
    else:
        current_state = before.iloc[-1]['new_state']

    # Transitions that fall inside the window
    inside = fsm_df[
        (fsm_df['ts_us'] > ts_start) & (fsm_df['ts_us'] < ts_end)
    ].copy()

    regions = []
    prev_t = ts_start
    for _, row in inside.iterrows():
        regions.append({
            'state':   current_state,
            't_start': prev_t,
            't_end':   row['ts_us'],
        })
        current_state = row['new_state']
        prev_t = row['ts_us']

    regions.append({
        'state':   current_state,
        't_start': prev_t,
        't_end':   ts_end,
    })
    return regions


# ── plotting ─────────────────────────────────────────────────────────────────

COORD_COLORS = ['#2196F3', '#4CAF50', '#F44336']   # blue, green, red
COORD_LABELS = ['p[0]', 'p[1]', 'p[2]']

# Colour cycle for FSM regions (background tint)
FSM_BG_COLORS = [
    '#FFF9C4', '#E8F5E9', '#E3F2FD', '#FCE4EC',
    '#F3E5F5', '#E0F7FA', '#FFF3E0', '#F9FBE7',
]


def plot(
    state_df: pd.DataFrame,
    regions: list[dict],
    ts_start: int,
    ts_end: int,
    output_path: str,
) -> None:
    t = state_df['ts_us'].values
    # Convert µs to seconds relative to start for readability
    t_sec = (t - ts_start) / 1e6

    fig, axes = plt.subplots(3, 1, figsize=(16, 9), sharex=True)
    fig.suptitle('Position Coordinates with FSM Regions', fontsize=14, fontweight='bold')

    # Unique FSM states → deterministic colour mapping
    unique_states = list(dict.fromkeys(r['state'] for r in regions))
    state_color = {
        s: FSM_BG_COLORS[i % len(FSM_BG_COLORS)]
        for i, s in enumerate(unique_states)
    }

    for ax, col, color, label in zip(axes, ['p[0]', 'p[1]', 'p[2]'],
                                     COORD_COLORS, COORD_LABELS):
        ax.plot(t_sec, state_df[col].values, color=color, linewidth=0.8,
                label=label)

        # FSM region shading + labels
        for region in regions:
            x0 = (region['t_start'] - ts_start) / 1e6
            x1 = (region['t_end']   - ts_start) / 1e6
            ax.axvspan(x0, x1, alpha=0.25,
                       color=state_color[region['state']], zorder=0)
            # Dashed orange vertical line at region boundary (skip very first)
            if region['t_start'] != ts_start:
                ax.axvline(x=x0, color='darkorange', linestyle='--',
                           linewidth=1.0, alpha=0.85, zorder=2)
            # State label — only on the top subplot to avoid clutter
            if ax is axes[0]:
                mid = (x0 + x1) / 2
                y_pos = ax.get_ylim()[1] if ax.get_ylim()[1] != 0 else 1
                # We'll reposition after autoscale; use transform instead
                ax.text(
                    mid, 1.01, region['state'],
                    transform=ax.get_xaxis_transform(),
                    ha='center', va='bottom',
                    fontsize=7.5, color='darkorange', fontweight='bold',
                    clip_on=False,
                )

        ax.set_ylabel(label, fontsize=10)
        ax.yaxis.set_minor_locator(AutoMinorLocator())
        ax.grid(True, which='major', linestyle='--', alpha=0.4)
        ax.grid(True, which='minor', linestyle=':', alpha=0.2)
        ax.legend(loc='upper right', fontsize=8)

    axes[-1].set_xlabel('Time since start (s)', fontsize=10)
    axes[-1].set_xlim(0, (ts_end - ts_start) / 1e6)

    # Legend for FSM states
    patches = [
        mpatches.Patch(
            facecolor=state_color[s], alpha=0.5, edgecolor='grey',
            label=s,
        )
        for s in unique_states
    ]
    fig.legend(
        handles=patches,
        title='FSM State', title_fontsize=9,
        loc='lower center', ncol=min(len(unique_states), 6),
        fontsize=8, framealpha=0.85,
        bbox_to_anchor=(0.5, -0.02),
    )

    plt.tight_layout(rect=[0, 0.04, 1, 1])
    fig.savefig(output_path, dpi=150, bbox_inches='tight')
    print(f"\n✓  Plot saved to: {output_path}")
    plt.show()


# ── main ─────────────────────────────────────────────────────────────────────

def main() -> None:
    print("=" * 60)
    print("  Position & FSM Region Plotter")
    print("=" * 60)

    # 1. Folder
    folder = prompt_folder()
    state_path = os.path.join(folder, 'eskf', 'State.csv')
    fsm_path   = os.path.join(folder, 'FsmTransition.csv')

    for path in (state_path, fsm_path):
        if not os.path.isfile(path):
            print(f"✗  File not found: {path}")
            sys.exit(1)

    # 2. Time window
    print("\nSpecify the time window to plot (from the ts_us column).")
    ts_start = prompt_timestamp("start")
    ts_end   = prompt_timestamp("end  ", reference=ts_start)

    # 3. Load data
    fsm_df   = load_fsm(fsm_path)
    state_df = load_state_chunked(state_path, ts_start, ts_end)

    # 4. Build FSM regions
    regions = build_fsm_regions(fsm_df, ts_start, ts_end, initial_state="INIT")
    print(f"\nFSM regions in window ({len(regions)}):")
    for r in regions:
        dur = (r['t_end'] - r['t_start']) / 1e6
        print(f"  [{r['t_start']}–{r['t_end']}]  {r['state']}  ({dur:.3f} s)")

    # 5. Plot
    output_path = os.path.join(folder, 'position_plot.png')
    plot(state_df, regions, ts_start, ts_end, output_path)


if __name__ == '__main__':
    main()