---
id: AA-1
title: Embed self-hosted robot CAD viewer
status: Done
assignee: []
created_date: '2026-10-15 19:40'
updated_date: '2026-10-15 22:12'
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
1. Produce a browser-compatible GLB asset from the committed STEP assembly using locally available conversion tooling. 2. Add an accessible, self-hosted interactive viewer and STEP download link to the 10/15/26 Dev Log entry. 3. Add a matching viewer or preview to the home page and shared responsive styles. 4. Validate the generated asset, HTML structure, and local page references.
<!-- SECTION:PLAN:END -->

## Implementation Notes

<!-- SECTION:NOTES:BEGIN -->
Converted CAD/Assembly 1.step to assets/robot-assembly.glb (24.8 MB) and vendored the model-viewer web component at js/model-viewer.min.js. Embedded the self-hosted viewer and STEP download links in devlog.html and index.html, with responsive shared styles in css/style.css. Validated both HTML files with Python HTMLParser and validated the GLB with gltf-validator: 0 errors, 0 warnings, 0 infos, 0 hints.

Browser validation: served the site locally and used Playwright Chromium to load both index.html and devlog.html. On each page, the self-hosted GLB returned HTTP 200, loaded in model-viewer, and passed simulated drag-to-rotate and wheel-to-zoom interaction checks with no browser errors.
<!-- SECTION:NOTES:END -->

## Final Summary

<!-- SECTION:FINAL_SUMMARY:BEGIN -->
Added a self-hosted interactive robot CAD viewer and original STEP downloads to the home page and 10/15/26 Dev Log entry. Verified the GLB structurally with gltf-validator (0 issues) and functionally with Playwright on both pages: local asset HTTP 200, successful load, drag rotation, and wheel zoom.
<!-- SECTION:FINAL_SUMMARY:END -->
