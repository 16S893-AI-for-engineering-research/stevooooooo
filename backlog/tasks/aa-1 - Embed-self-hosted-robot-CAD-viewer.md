---
id: AA-1
title: Embed self-hosted robot CAD viewer
status: Done
assignee: []
created_date: '2026-10-15 19:40'
updated_date: '2026-10-18 01:59'
labels: []
dependencies: []
type: feature
ordinal: 1000
---

## Description

<!-- SECTION:DESCRIPTION:BEGIN -->
The robot assembly is available in the repository only as a STEP file, which browsers cannot display interactively. Add a self-hosted browser-viewable version to the Dev Log post dated 10/15/26 and the home page so visitors can rotate and zoom the current robot CAD system without an external viewer service.
<!-- SECTION:DESCRIPTION:END -->

## Acceptance Criteria
<!-- AC:BEGIN -->
- [x] #1 The 10/15/26 Dev Log entry embeds an interactive self-hosted 3D robot model.
- [x] #2 The home page includes an interactive self-hosted 3D robot model or model preview.
- [x] #3 The original STEP assembly remains downloadable from the website.
- [x] #4 The embedded model supports mouse/touch rotation and zoom without external viewer services.
<!-- AC:END -->

## Implementation Plan

<!-- SECTION:PLAN:BEGIN -->
1. Replace the prior Z-axis display rotation with a 180-degree X-axis rotation in both CAD viewer embeds. 2. Verify both pages load the local model and retain interactive controls. 3. Push the corrected orientation.
<!-- SECTION:PLAN:END -->

## Implementation Notes

<!-- SECTION:NOTES:BEGIN -->
Converted CAD/Assembly 1.step to assets/robot-assembly.glb (24.8 MB) and vendored the model-viewer web component at js/model-viewer.min.js. Embedded the self-hosted viewer and STEP download links in devlog.html and index.html, with responsive shared styles in css/style.css. Validated both HTML files with Python HTMLParser and validated the GLB with gltf-validator: 0 errors, 0 warnings, 0 infos, 0 hints.

Browser validation: served the site locally and used Playwright Chromium to load both index.html and devlog.html. On each page, the self-hosted GLB returned HTTP 200, loaded in model-viewer, and passed simulated drag-to-rotate and wheel-to-zoom interaction checks with no browser errors.

Reopened at the user's request to optimize the browser model. Confirmed scope: replace the existing GLB on both pages; prioritize integrated-graphics interaction; target 2–5 MB; do not retain the full-detail GLB; retain the original STEP download.

Optimization result: replaced the 24,777,212-byte GLB with a 1,612,240-byte GLB (93.5% smaller). Coarser 2.0-unit tessellation and face merging reduced the source conversion from 42,424 primitives to 673; glTF optimization then merged the web asset to one mesh and one primitive, with Meshopt compression. Added a self-hosted Meshopt initialization signal for model-viewer. Structural validation reported 0 errors and 0 warnings (one expected informational note: validator does not implement EXT_meshopt_compression). Playwright loaded both pages using the local GLB and local Meshopt setup script with HTTP 200; drag rotation and wheel zoom passed without browser errors.

Reopened at the user's request to correct the initial upside-down display orientation. Confirmed approach: rotate the displayed model by 180 degrees in both viewers; leave the GLB and downloadable STEP unchanged.

Applied orientation="0deg 0deg 180deg" to the model viewer in both index.html and devlog.html. Playwright verified that both pages load the local model, expose the configured 180-degree orientation, and retain drag rotation without browser errors. HTML parsing and git diff whitespace checks passed.

Reopened at the user's request because the Z-axis rotation did not resolve the visual inversion. Confirmed approach: use a 180-degree X-axis rotation in both viewers.

Replaced the unsuccessful Z-axis display rotation with orientation="180deg 0deg 0deg" in both viewers. Playwright verified that each page loaded the local model with the configured X-axis orientation and no browser errors; HTML parsing and git diff whitespace checks passed.
<!-- SECTION:NOTES:END -->

## Final Summary

<!-- SECTION:FINAL_SUMMARY:BEGIN -->
Replaced the prior Z-axis rotation with a 180-degree X-axis rotation in both CAD viewers to correct the actual model inversion. The model asset and STEP download remain unchanged; both embeds load successfully.
<!-- SECTION:FINAL_SUMMARY:END -->
