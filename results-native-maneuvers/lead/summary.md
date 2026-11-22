# Native validation

Mode: `lead`

Status: **PASSED**

Source revision: `619b2f4e065bb63e62c3bd849a90383dc942ec23`; dirty at run start: `False`.

Started (UTC): 2026-11-22T12:21:30.030582+00:00

Wall duration: 208.93 s.

[Raw validation](validation.json) · [Source provenance](metadata.json)

| Scenario | Status | Duration (sim s) | Min footprint gap bound (m) | Final state |
|---|---|---:|---:|---|
| [static](static/result.json) | passed | 36.00 | 5.084 | lead_vehicle |
| [moving](moving/result.json) | passed | 60.00 | 4.760 | driving |
| [braking](braking/result.json) | passed | 66.00 | 3.851 | lead_vehicle |
| [loss](loss/result.json) | passed | 10.00 | 14.086 | stale_lead_estimate |
