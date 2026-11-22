# Native validation

Mode: `lead`

Status: **PASSED**

Source revision: `270f00577eb3789645540a294f1711ac9695a524`; dirty at run start: `False`.

Started (UTC): 2026-11-22T11:54:54.381899+00:00

Wall duration: 136.88 s.

[Raw validation](validation.json) · [Source provenance](metadata.json)

| Scenario | Status | Duration (sim s) | Min footprint gap bound (m) | Final state |
|---|---|---:|---:|---|
| [static](static/result.json) | passed | 36.00 | 5.082 | lead_vehicle |
| [moving](moving/result.json) | passed | 60.00 | 4.761 | driving |
| [loss](loss/result.json) | passed | 10.00 | 14.128 | stale_lead_estimate |
