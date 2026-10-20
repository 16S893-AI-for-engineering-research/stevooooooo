---
id: AA-6
title: USB bandwidth arbiter enforcing single-camera streaming
status: To Do
assignee: []
created_date: '2026-10-20 21:08'
updated_date: '2026-10-20 21:08'
labels: []
milestone: m-0
dependencies:
  - AA-5
documentation:
  - >-
    backlog/docs/specs/doc-1 -
    Inchworm-Print-Crawl-Prototype-—-Hardware-and-System-Reference.md
priority: high
type: feature
ordinal: 6000
---

## Description

<!-- SECTION:DESCRIPTION:BEGIN -->
All arms and cameras share one USB 2.0 uplink through a single dock, so only one camera may transfer data at a time. This is a hard system invariant: if two cameras stream concurrently, frames drop or the bus stalls, which can corrupt a laser scan or an insertion policy observation mid-motion. The constraint must be enforced structurally so no later task can violate it by accident.

The autonomy loop will switch cameras repeatedly (wrist camera during insertion, scanner camera during leveling), so acquisition must be cheap, reliable and must fully close the previous device rather than just pausing it.

See doc-1 section 6.
<!-- SECTION:DESCRIPTION:END -->

## Acceptance Criteria
<!-- AC:BEGIN -->
- [ ] #1 A single process-wide and cross-process arbiter grants exclusive camera access; a second acquisition attempt while a camera is held either blocks with a timeout or fails explicitly, never silently proceeds
- [ ] #2 Acquisition is exposed as a context manager (or equivalent) that opens the device on enter and fully closes and releases it on exit, including on exception
- [ ] #3 Releasing a camera verifiably closes the underlying device handle rather than pausing the stream
- [ ] #4 A crashed or killed holder does not permanently deadlock the arbiter; stale locks are detected and recovered with a logged warning
- [ ] #5 Arm and relay serial traffic is explicitly permitted concurrently with one camera stream and is not gated by the arbiter
- [ ] #6 Tests prove mutual exclusion, timeout behaviour, exception-path release and stale-lock recovery against the mock backend
<!-- AC:END -->
