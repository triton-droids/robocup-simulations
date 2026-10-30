# Field definitions

Copied unchanged from the HSL rule book repository:
<https://github.com/RoboCup-HumanoidSoccerLeague/HSL-Rules>, tag `rules-2026-v1.1.1` (commit `06c44d4`),
path `figs/FieldGenerator/`.

| File | sha256 |
|---|---|
| `field_hsl_s_2026.json` | `ca8c254b41d0dc12685a721052ee76f68ea917bc01fe28fdee77c91e42ef84d6` |
| `field_hsl_m_2026.json` | `835f7d3e0005f9fe6bff0371316fffe4c3065dbc489d812cfc795a2b19383e00` |
| `field_hsl_l_2026.json` | `e18e466a74fe8c8f07377edeaae25f2d1d3b4303536a730b8f867341b5408379` |

## Convention (from `generateField.py`)

- **Line-center distances:** `length`, `width`, the area lengths and widths, `penaltyMarkDistance`,
  `centerCircleDiameter` and `borderStripWidth` are measured between line **centers**. The rule book's table quotes
  outside-edge dimensions, which are larger by one line width.
- **Corner arcs:** `cornerArcRadius` is the arc's centerline radius around the field's **outer** corner.

## Known discrepancy in the L-Field file

- Every L value except `length` and `width` matches the rule table using the 0.12 m line width. For example,
  `goalAreaLength` 0.88 = 1.0 − 0.12.
- `length` and `width` are 21.95 and 13.95. That implies outside dimensions of 22.07 × 14.07 m, not the table's
  22 × 14 m. They look like they were computed with the 0.05 m line width.
- The files are kept unchanged. `test_field.py` pins the discrepancy so a future rules update shows up.
- Real venues vary within the ranges the rules allow. **Measure the actual field at an event and load a venue JSON**
  with `FieldModel.from_json()`.
