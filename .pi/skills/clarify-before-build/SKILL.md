---
name: clarify-before-build
description: Use whenever the user asks the agent to BUILD something physical or tangible-feeling — a website, simulation, animation, game, UI, hardware layout, or any artifact with visual/behavioral form. Before writing any code, ask enough clarifying questions to pin down exactly what "done" looks like, then restate the plan and get confirmation. Prevents guessing at ambiguous requirements (layout, motion, style, interactions, scope) that leads to rework.
---

# Clarify Before Build

## When this applies

Trigger this skill any time a request asks the agent to *build*, *create*,
*make*, or *animate* something with a physical or visible form, including:

- Websites / web pages / UI components
- Animations or motion graphics
- Simulations (physics, games, models)
- Diagrams, charts, or visual layouts
- Physical/hardware designs described in text (robots, mechanisms, CAD-ish asks)
- Anything where "it works" is not enough — it also has to *look* or *behave*
  a specific way

Do **not** trigger this for purely logical/backend tasks with no visual or
behavioral ambiguity (e.g., "write a function that sorts this list",
"fix this bug", "add a git commit").

## Why

Physical/visual builds fail silently: the agent can produce code that is
functionally correct but looks nothing like what the user imagined (wrong
motion, wrong layout, wrong tone, wrong scope). Unlike a bug, there's no
error message — just a user who has to describe the mismatch in words after
the fact, which is slower and more frustrating than asking up front.

## Procedure

1. **Do not start writing code yet.** First, identify what is ambiguous
   about the request. At minimum consider:
   - **Visual style / theme** — colors, tone, references, "looks like X"?
   - **Layout / structure** — how many pieces, how are they arranged, what's
     on screen at once?
   - **Motion / behavior** — what moves, how fast, triggered by what, does
     it loop, ease, bounce, respond to interaction?
   - **Scope** — is this the whole thing or one piece of a larger ask? What
     is explicitly out of scope?
   - **Constraints** — platform, framework, file format, performance limits,
     accessibility, existing code it must fit into.
   - **Success criteria** — how will the user know it's right when they see
     it? What would make them reject it?

2. **Ask targeted clarifying questions** covering the ambiguous points above.
   Keep the list tight — 3 to 6 questions, not an exhaustive interview.
   Prefer questions with a natural default so the user can just confirm or
   correct ("I'm planning X for the animation speed — is that about right,
   or should it be snappier/slower?").

3. **If the user's answers are vague or partial**, ask a tight follow-up
   round rather than filling gaps with silent assumptions. Two short rounds
   of questions is better than one wrong build.

4. **Restate the plan in plain language** before writing any code: what it
   will look like, what will move and how, what's included and excluded.
   Ask for a go-ahead ("Does that match what you have in mind?").

5. **Only after confirmation**, proceed to build. If something is still
   ambiguous mid-build (e.g., an edge case the questions didn't cover), stop
   and ask rather than guessing and continuing.

6. **After delivering**, explicitly invite correction on the physical/visual
   result specifically (not just "let me know if there are bugs") — e.g.,
   "Take a look and tell me if the motion/layout/style needs adjusting."

## What NOT to do

- Do not immediately start writing HTML/CSS/JS, simulation code, or diagrams
  based on a one-line request like "build me an animated site" without
  asking anything first.
- Do not ask so many questions that it becomes a wall of text — group related
  questions and lead with sensible defaults.
- Do not treat this as optional when the user says "just build something" —
  gently push back with 1-2 questions to establish minimum viable direction
  (theme, tone, scope) even if they want you to take creative liberty on the
  rest. Confirm what "creative liberty" is allowed to vary.

## Example

**User:** "Build me a simulation of planets orbiting the sun."

**Bad (skip clarification):** Immediately writes a canvas/JS orbital
simulation with arbitrary planet count, colors, and speeds.

**Good (this skill):**
> Before I build this, a few quick questions:
> 1. Real solar system (accurate-ish relative sizes/speeds) or a simplified/
>    stylized version — any number of planets, exaggerated motion?
> 2. Should it be interactive (click/drag to change speed, zoom) or just
>    play automatically?
> 3. Visual style — realistic textures, flat/minimal shapes, or something
>    else (e.g., neon/retro)?
> 4. Standalone page, or does it need to fit into an existing site/theme?
>
> My default guess: simplified 2D top-down view, ~5-6 planets, flat colored
> circles, auto-playing orbits, standalone HTML/CSS/JS page. Let me know
> what to change before I start.
