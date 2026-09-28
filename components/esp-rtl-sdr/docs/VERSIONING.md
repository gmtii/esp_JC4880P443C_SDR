# Versioning and release stages

`esp_rtl_sdr` uses Semantic Versioning: `MAJOR.MINOR.PATCH[-stageN]`.

- **alphaN** — incomplete experimental work. APIs and behavior may change.
- **betaN** — the intended feature set is substantially present, but API or
  hardware behavior may still change as validation expands.
- **rcN** — a release candidate for the exact stable `MAJOR.MINOR.PATCH`.
  Scope and public API are frozen; only release-blocking fixes, evidence, and
  release documentation may change. Any code fix increments `N`.
- **No suffix** — the stable release.

Patch releases contain backward-compatible fixes. Minor releases add
backward-compatible capability. Major releases may break the public API.

Private test builds do not create another version stage. Identify them by the
reported version, full commit SHA, artifact SHA-256, and an optional CI/build
number. Tags are created only for published releases.

The `0.8.0-rc3` train is an RC because the `0.8.0` public driver contract and
supported scope are frozen. Provisional device profiles remain explicitly
provisional capability within that contract; they do not turn an RC back into
a product beta.
