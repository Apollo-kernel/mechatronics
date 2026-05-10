const deck = document.querySelector("#deck");
const slides = Array.from(document.querySelectorAll(".slide"));
const dots = Array.from(document.querySelectorAll("[data-goto]"));
const prevButton = document.querySelector("#prevSlide");
const nextButton = document.querySelector("#nextSlide");
const slideNow = document.querySelector("#slideNow");
const slideTotal = document.querySelector("#slideTotal");
const progressBar = document.querySelector("#progressBar");

let activeIndex = 0;
slideTotal.textContent = String(slides.length).padStart(2, "0");

function clampIndex(index) {
  return Math.min(Math.max(index, 0), slides.length - 1);
}

function goToSlide(index) {
  const targetIndex = clampIndex(index);
  const target = slides[targetIndex];
  setActive(targetIndex);
  deck.scrollTo({ top: target.offsetTop, behavior: "smooth" });
}

function setActive(index) {
  activeIndex = clampIndex(index);
  slideNow.textContent = String(activeIndex + 1).padStart(2, "0");
  progressBar.style.width = `${((activeIndex + 1) / slides.length) * 100}%`;

  dots.forEach((dot, dotIndex) => {
    dot.classList.toggle("is-active", dotIndex === activeIndex);
    dot.setAttribute("aria-current", dotIndex === activeIndex ? "step" : "false");
  });

  prevButton.disabled = activeIndex === 0;
  nextButton.disabled = activeIndex === slides.length - 1;
}

const observer = new IntersectionObserver(
  (entries) => {
    const visible = entries
      .filter((entry) => entry.isIntersecting)
      .sort((a, b) => b.intersectionRatio - a.intersectionRatio)[0];

    if (!visible) return;
    setActive(slides.indexOf(visible.target));
  },
  {
    root: deck,
    threshold: [0.55, 0.7, 0.9],
  },
);

slides.forEach((slide) => observer.observe(slide));

dots.forEach((dot) => {
  dot.addEventListener("click", () => {
    goToSlide(Number(dot.dataset.goto));
  });
});

prevButton.addEventListener("click", () => goToSlide(activeIndex - 1));
nextButton.addEventListener("click", () => goToSlide(activeIndex + 1));

window.addEventListener("keydown", (event) => {
  const tag = document.activeElement?.tagName;
  if (tag === "INPUT" || tag === "TEXTAREA" || tag === "SELECT") return;

  if (event.key === "ArrowRight" || event.key === "PageDown" || event.key === " ") {
    event.preventDefault();
    goToSlide(activeIndex + 1);
  }

  if (event.key === "ArrowLeft" || event.key === "PageUp") {
    event.preventDefault();
    goToSlide(activeIndex - 1);
  }

  if (event.key === "Home") {
    event.preventDefault();
    goToSlide(0);
  }

  if (event.key === "End") {
    event.preventDefault();
    goToSlide(slides.length - 1);
  }
});

deck.addEventListener("scroll", () => {
  const maxScroll = deck.scrollHeight - deck.clientHeight;
  if (maxScroll <= 0) return;
  const scrollProgress = deck.scrollTop / maxScroll;
  progressBar.style.width = `${Math.max(
    ((activeIndex + 1) / slides.length) * 100,
    scrollProgress * 100,
  )}%`;
});

setActive(0);
