// Builds a Warren-style truss spanning the container's width, then animates
// a two-segment inchworm-style robot crawling along the TOP chord of the
// truss. Click the robot to trigger the Easter egg: the front (right,
// direction-of-travel) segment releases its grip on the truss while the
// rear (left) segment stays anchored, and both segments swing together as
// a rigid unit into a big exaggerated wave, then the front re-grips and
// crawling resumes.
(function () {
  const container = document.getElementById("truss-container");
  if (!container) return;

  const NS = "http://www.w3.org/2000/svg";
  const VB_W = 1200;
  const VB_H = 220;

  function el(tag, attrs) {
    const node = document.createElementNS(NS, tag);
    for (const k in attrs) node.setAttribute(k, attrs[k]);
    return node;
  }

  const svg = el("svg", {
    id: "truss-svg",
    viewBox: `0 0 ${VB_W} ${VB_H}`,
    preserveAspectRatio: "none",
    "aria-hidden": "true"
  });
  container.appendChild(svg);

  // ---- defs (gradients) ----
  const defs = el("defs", {});
  const grad = el("linearGradient", { id: "wormGradient", x1: "0%", y1: "0%", x2: "0%", y2: "100%" });
  grad.appendChild(el("stop", { offset: "0%", "stop-color": "#8fe9ff" }));
  grad.appendChild(el("stop", { offset: "100%", "stop-color": "#2c8fb0" }));
  defs.appendChild(grad);
  svg.appendChild(defs);

  // ---- static truss geometry ----
  const trussGroup = el("g", { id: "truss-static" });
  svg.appendChild(trussGroup);

  const yTop = 100;
  const yBottom = 180;
  const bayWidth = 60;
  const bays = Math.ceil(VB_W / bayWidth) + 2;

  trussGroup.appendChild(el("line", { class: "truss-line", x1: -bayWidth, y1: yBottom, x2: VB_W + bayWidth, y2: yBottom }));
  trussGroup.appendChild(el("line", { class: "truss-line", x1: -bayWidth, y1: yTop, x2: VB_W + bayWidth, y2: yTop }));

  for (let i = -1; i <= bays; i++) {
    const x = i * bayWidth;
    trussGroup.appendChild(el("line", { class: "truss-brace", x1: x, y1: yTop, x2: x, y2: yBottom }));
    trussGroup.appendChild(el("line", {
      class: "truss-brace",
      x1: x, y1: (i % 2 === 0) ? yTop : yBottom,
      x2: x + bayWidth, y2: (i % 2 === 0) ? yBottom : yTop
    }));
    trussGroup.appendChild(el("circle", { class: "truss-node", cx: x, cy: yTop, r: 3 }));
    trussGroup.appendChild(el("circle", { class: "truss-node", cx: x, cy: yBottom, r: 3 }));
  }

  // ---- inchworm robot: two rigid segments joined at a hip ----
  const wormGroup = el("g", { id: "worm" });
  svg.appendChild(wormGroup);

  const hitArea = el("rect", { id: "worm-hit", fill: "transparent", x: 0, y: 0, width: 1, height: 1 });
  wormGroup.appendChild(hitArea);

  const segRear = el("line", { id: "seg-rear", class: "worm-seg" }); // rear foot -> hip
  const segFront = el("line", { id: "seg-front", class: "worm-seg" }); // hip -> front foot
  wormGroup.appendChild(segRear);
  wormGroup.appendChild(segFront);

  const hipJoint = el("circle", { id: "hip-joint", class: "worm-hip", r: 5 });
  wormGroup.appendChild(hipJoint);

  const footRear = el("circle", { id: "foot-rear", class: "worm-foot", r: 6 });
  const footFront = el("circle", { id: "foot-front", class: "worm-foot", r: 6 });
  wormGroup.appendChild(footRear);
  wormGroup.appendChild(footFront);

  const eyeGroup = el("g", { id: "worm-eyes" });
  eyeGroup.appendChild(el("circle", { class: "worm-eye", cx: 0, cy: 0, r: 2.4 }));
  eyeGroup.appendChild(el("circle", { class: "worm-eye", cx: 7, cy: 0, r: 2.4 }));
  wormGroup.appendChild(eyeGroup);

  // ---------------- animation state ----------------
  const yTruss = yTop; // the robot crawls along the TOP chord
  const STEP = 70;
  const GAP0 = 55; // rest distance between feet (also each segment's length)
  const CYCLE_MS = 2200;
  const ARCH_MAX = 42;
  const LIFT_MAX = 11;
  const CLEARANCE = 16;
  const WRAP_SHIFT = VB_W + 160;
  const WAVE_MS = 1900;

  let rearBase = -GAP0 - 40;
  let frontBase = rearBase + GAP0;
  let cycleStart = null;
  let isPaused = false;
  let waveActive = false;
  let waveStart = null;

  // frozen pose captured the instant the robot is clicked, used both to
  // render the wave and to resume crawling from the same spot afterward
  let frozenRearX = 0;
  let frozenFrontX = 0;

  function ease(t) { return t * t * (3 - 2 * t); }

  function hipPosition(rx, fx, arch) {
    return { x: (rx + fx) / 2, y: yTruss - CLEARANCE - arch };
  }

  function drawSegments(rx, ry, fx, fy, hip) {
    segRear.setAttribute("x1", rx);
    segRear.setAttribute("y1", ry);
    segRear.setAttribute("x2", hip.x);
    segRear.setAttribute("y2", hip.y);

    segFront.setAttribute("x1", hip.x);
    segFront.setAttribute("y1", hip.y);
    segFront.setAttribute("x2", fx);
    segFront.setAttribute("y2", fy);

    hipJoint.setAttribute("cx", hip.x);
    hipJoint.setAttribute("cy", hip.y);

    footRear.setAttribute("cx", rx);
    footRear.setAttribute("cy", ry);
    footFront.setAttribute("cx", fx);
    footFront.setAttribute("cy", fy);

    eyeGroup.setAttribute("transform", `translate(${fx - 8}, ${hip.y - 10})`);
  }

  function updateHitArea(minX, maxX, minY, maxY) {
    hitArea.setAttribute("x", minX - 20);
    hitArea.setAttribute("y", minY - 20);
    hitArea.setAttribute("width", (maxX - minX) + 40);
    hitArea.setAttribute("height", (maxY - minY) + 40);
  }

  function render(now) {
    if (!cycleStart) cycleStart = now;

    if (waveActive) {
      renderWave(now);
      requestAnimationFrame(render);
      return;
    }

    if (isPaused) {
      requestAnimationFrame(render);
      return;
    }

    const elapsed = now - cycleStart;
    const p = (elapsed % CYCLE_MS) / CYCLE_MS;
    const cyclesCompleted = Math.floor(elapsed / CYCLE_MS);
    const rb = rearBase + cyclesCompleted * STEP;
    const fb = frontBase + cyclesCompleted * STEP;

    let frontX, rearX;
    if (p < 0.5) {
      const t = ease(p / 0.5);
      frontX = fb + STEP * t;
      rearX = rb;
    } else {
      const t = ease((p - 0.5) / 0.5);
      frontX = fb + STEP;
      rearX = rb + STEP * t;
    }

    const archHeight = p < 0.5 ? 0 : ARCH_MAX * Math.sin(Math.PI * (p - 0.5) / 0.5);
    const liftFront = p < 0.5 ? LIFT_MAX * Math.sin(Math.PI * (p / 0.5)) : 0;
    const liftRear = p < 0.5 ? 0 : LIFT_MAX * Math.sin(Math.PI * (p - 0.5) / 0.5);

    // note: lifting a foot means moving it UP, i.e. to a smaller y value
    const frontY = yTruss - liftFront;
    const rearY = yTruss - liftRear;

    const hip = hipPosition(rearX, frontX, archHeight);
    drawSegments(rearX, rearY, frontX, frontY, hip);
    updateHitArea(rearX, frontX, hip.y - 40, yTruss + 10);

    if (frontX > VB_W + 100) {
      rearBase -= WRAP_SHIFT;
      frontBase -= WRAP_SHIFT;
    }

    requestAnimationFrame(render);
  }

  // ---- Easter egg: rear segment lets go, both segments wave together,
  // pivoting from the still-anchored front foot ----
  // ---- Easter egg: front (right) segment lets go, swinging out from the
  // still-anchored rear (left) foot to wave, exaggerated and floppy ----
  function renderWave(now) {
    if (!waveStart) waveStart = now;
    const u = Math.min((now - waveStart) / WAVE_MS, 1);

    const anchor = { x: frozenRearX, y: yTruss };
    const restLean = 30;    // degrees off vertical at rest, leaning forward over the front foot
    const raisedLean = 100; // degrees off vertical when fully raised — past horizontal for a big wave

    let lean;
    if (u < 0.15) {
      lean = restLean + (raisedLean - restLean) * ease(u / 0.15);
    } else if (u < 0.85) {
      const w = (u - 0.15) / 0.7;
      lean = raisedLean + 30 * Math.sin(w * Math.PI * 4.5);
    } else {
      const w = (u - 0.85) / 0.15;
      lean = raisedLean + (restLean - raisedLean) * ease(w);
    }

    const rad = (lean * Math.PI) / 180;
    const segLen = GAP0 / 2 + 8;

    // both segments swing together as one rigid unit, pivoting at the anchor,
    // extending out to the right (direction of travel) as they wave
    const hipX = anchor.x + Math.sin(rad) * segLen;
    const hipY = anchor.y - Math.cos(rad) * segLen;
    const tipX = hipX + Math.sin(rad) * segLen;
    const tipY = hipY - Math.cos(rad) * segLen;

    drawSegments(anchor.x, anchor.y, tipX, tipY, { x: hipX, y: hipY });
    updateHitArea(Math.min(tipX, anchor.x), Math.max(tipX, anchor.x), tipY - 40, anchor.y + 10);

    if (u >= 1) {
      waveActive = false;
      waveStart = null;
      isPaused = false;
      // resume a fresh crawl cycle from exactly where the feet were when clicked
      rearBase = frozenRearX;
      frontBase = frozenFrontX;
      cycleStart = performance.now();
    }
  }

  function triggerWave() {
    if (waveActive || isPaused) return;

    frozenRearX = parseFloat(footRear.getAttribute("cx"));
    frozenFrontX = parseFloat(footFront.getAttribute("cx"));

    // snap both feet down onto the truss for a clean, grounded wave pose
    const hip = hipPosition(frozenRearX, frozenFrontX, 0);
    drawSegments(frozenRearX, yTruss, frozenFrontX, yTruss, hip);

    isPaused = true;
    waveActive = true;
    waveStart = null;
  }

  wormGroup.addEventListener("click", triggerWave);
  wormGroup.style.cursor = "pointer";

  requestAnimationFrame(render);
})();
