---
id: AA-1
title: Embed self-hosted robot CAD viewer
status: Done
assignee: []
created_date: '2026-10-15 19:40'
updated_date: '2026-10-15 22:30'
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
1. Reconvert the committed STEP assembly using coarser tessellation and merged faces to dramatically reduce draw calls. 2. Compress the replacement GLB to a 2–5 MB web target, replacing the existing full-detail generated viewer asset while retaining the original STEP download. 3. Validate GLB structure, file size, and browser loading and camera interaction on both pages.
<!-- SECTION:PLAN:END -->

## Implementation Notes

<!-- SECTION:NOTES:BEGIN -->
Converted CAD/Assembly 1.step to assets/robot-assembly.glb (24.8 MB) and vendored the model-viewer web component at js/model-viewer.min.js. Embedded the self-hosted viewer and STEP download links in devlog.html and index.html, with responsive shared styles in css/style.css. Validated both HTML files with Python HTMLParser and validated the GLB with gltf-validator: 0 errors, 0 warnings, 0 infos, 0 hints.

Browser validation: served the site locally and used Playwright Chromium to load both index.html and devlog.html. On each page, the self-hosted GLB returned HTTP 200, loaded in model-viewer, and passed simulated drag-to-rotate and wheel-to-zoom interaction checks with no browser errors.

Reopened at the user's request to optimize the browser model. Confirmed scope: replace the existing GLB on both pages; prioritize integrated-graphics interaction; target 2–5 MB; do not retain the full-detail GLB; retain the original STEP download.

Optimization result: replaced the 24,777,212-byte GLB with a 1,612,240-byte GLB (93.5% smaller). Coarser 2.0-unit tessellation and face merging reduced the source conversion from 42,424 primitives to 673; glTF optimization then merged the web asset to one mesh and one primitive, with Meshopt compression. Added a self-hosted Meshopt initialization signal for model-viewer. Structural validation reported 0 errors and 0 warnings (one expected informational note: validator does not implement EXT_meshopt_compression). Playwright loaded both pages using the local GLB and local Meshopt setup script with HTTP 200; drag rotation and wheel zoom passed without browser errors.
<!-- SECTION:NOTES:END -->

## Final Summary

<!-- SECTION:FINAL_SUMMARY:BEGIN -->
Replaced the viewer model with a 1.61 MB web-optimized GLB, down from 24.78 MB (93.5% reduction). The model now renders as one mesh/primitive rather than 42,424 primitives, with coarser tessellation, merged geometry, and Meshopt compression. Verified structural validity and local browser loading, rotation, and zoom on both pages.
<!-- SECTION:FINAL_SUMMARY:END -->
