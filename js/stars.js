// Generates a lightweight twinkling starfield behind every page.
(function () {
  const layer = document.getElementById("stars");
  if (!layer) return;

  const COUNT = Math.min(140, Math.floor((window.innerWidth * window.innerHeight) / 9000));

  for (let i = 0; i < COUNT; i++) {
    const star = document.createElement("div");
    star.className = "star";
    const size = Math.random() * 2 + 0.5;
    star.style.width = `${size}px`;
    star.style.height = `${size}px`;
    star.style.left = `${Math.random() * 100}%`;
    star.style.top = `${Math.random() * 100}%`;
    star.style.animationDuration = `${2 + Math.random() * 4}s`;
    star.style.animationDelay = `${Math.random() * 4}s`;
    layer.appendChild(star);
  }

  const drift = document.createElement("div");
  drift.className = "nebula-drift";
  document.body.appendChild(drift);
})();
