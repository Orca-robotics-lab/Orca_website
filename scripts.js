/* ORCA Robotics — small client-side helpers */

// Scroll-fade reveal — position-based (robust across all environments, no IO dependency)
(function () {
  // Stagger delays for cards inside .reveal-group
  document.querySelectorAll(".reveal-group").forEach(function (group) {
    Array.from(group.children).forEach(function (child, i) {
      child.classList.add("reveal");
      child.style.transitionDelay = (i * 60) + "ms";
    });
  });

  var els = Array.prototype.slice.call(document.querySelectorAll(".reveal"));

  function revealInView() {
    var vh = window.innerHeight || document.documentElement.clientHeight;
    for (var i = els.length - 1; i >= 0; i--) {
      var el = els[i];
      var r = el.getBoundingClientRect();
      // reveal once the element's top enters the lower 92% of the viewport
      if (r.top < vh * 0.92 && r.bottom > 0) {
        el.classList.add("is-in");
        // Guarantee the final state lands even where transitions are throttled
        // (offscreen/headless). In a normal tab the fade has already finished by
        // now, so clearing the transition is a harmless no-op.
        (function (node) {
          setTimeout(function () { node.style.transition = "none"; }, 720);
        })(el);
        els.splice(i, 1);
      }
    }
  }

  var ticking = false;
  function onScroll() {
    revealInView();
    if (ticking) return;
    ticking = true;
    window.requestAnimationFrame(function () { revealInView(); ticking = false; });
  }

  revealInView();
  window.addEventListener("scroll", onScroll, { passive: true });
  window.addEventListener("resize", onScroll, { passive: true });
  window.addEventListener("load", function () { setTimeout(revealInView, 60); });

  // Backup poll — catches environments where scroll events / rAF are throttled
  // (offscreen iframes, headless capture). Stops once everything is revealed.
  var poll = setInterval(function () {
    revealInView();
    if (els.length === 0) clearInterval(poll);
  }, 400);
})();

// Forms — send via Web3Forms (no backend needed), keep inline confirmation
(function () {
  // ─── Paste your free Web3Forms access key here ───────────────
  // Get one in 30s at https://web3forms.com (enter info@orcarobotics.in,
  // they email you a key). Until then forms just show the thank-you message.
  var WEB3FORMS_KEY = "b432d8a3-5c33-46e3-aba0-f2ce34dfcef7";

  document.querySelectorAll("form[data-enquiry]").forEach(function (form) {
    form.addEventListener("submit", function (e) {
      e.preventDefault();

      // Require all fields to be filled in (message stays optional via its
      // markup — it carries no `required` attribute). Native constraint
      // validation surfaces the browser's "please fill this in" prompts.
      if (typeof form.reportValidity === "function" && !form.reportValidity()) {
        return;
      }

      var confirm = form.querySelector(".form-confirm");
      var btn = form.querySelector('button[type="submit"]');

      function showConfirm() {
        if (confirm) confirm.classList.add("is-visible");
        form.reset();
      }

      // No key set yet → keep old behaviour (just confirm, send nothing)
      if (!WEB3FORMS_KEY || WEB3FORMS_KEY === "YOUR-ACCESS-KEY-HERE") {
        showConfirm();
        return;
      }

      var data = Object.fromEntries(new FormData(form).entries());
      data.access_key = WEB3FORMS_KEY;
      data.subject =
        "New enquiry from " + document.title + " — " + (data.name || "website");
      data.from_name = "ORCA Robotics Website";

      var origLabel = btn ? btn.innerHTML : "";
      if (btn) {
        btn.disabled = true;
        btn.innerHTML = "Sending\u2026";
      }

      fetch("https://api.web3forms.com/submit", {
        method: "POST",
        headers: {
          "Content-Type": "application/json",
          Accept: "application/json",
        },
        body: JSON.stringify(data),
      })
        .then(function (r) {
          return r.json();
        })
        .then(function (res) {
          if (res && res.success) {
            showConfirm();
          } else {
            window.alert(
              "Sorry, your enquiry could not be sent. Please email info@orcarobotics.in directly."
            );
          }
        })
        .catch(function () {
          window.alert(
            "Sorry, your enquiry could not be sent. Please email info@orcarobotics.in directly."
          );
        })
        .finally(function () {
          if (btn) {
            btn.disabled = false;
            btn.innerHTML = origLabel;
          }
        });
    });
  });
})();

// Workshop "Enquire" deep-link → preselect dropdown
(function () {
  document.querySelectorAll("[data-prefill-workshop]").forEach(function (link) {
    link.addEventListener("click", function () {
      const val = link.getAttribute("data-prefill-workshop");
      const sel = document.querySelector("select[name=workshop]");
      if (sel) {
        sel.value = val;
      }
    });
  });
})();

// Mobile hamburger (very lightweight: toggle a class)
(function () {
  const btn = document.querySelector(".nav__hamburger");
  if (!btn) return;
  btn.addEventListener("click", function () {
    document.body.classList.toggle("nav-open");
  });
})();



// Join Community modal — injected once, opened by any [data-community-open]
(function () {
  // --- Update these links to point at your real community invites ---
  var CHANNELS = [
    {
      name: "WhatsApp",
      url: "https://chat.whatsapp.com/IxVYYtXWHZ10hDHsYYKpIW",
    },
    {
      name: "Discord",
      url: "https://discord.gg/nh2f2kx6",
    },
    {
      name: "Instagram",
      url: "https://instagram.com/orca.robotics",
    },
  ];

  var LEAD =
    "Connect with fellow builders, share your projects, ask questions, and learn alongside a growing community of robotics enthusiasts.";

  function qr(url) {
    return (
      "https://api.qrserver.com/v1/create-qr-code/?size=240x240&margin=0&data=" +
      encodeURIComponent(url)
    );
  }

  function shortLabel(url) {
    return url.replace(/^https?:\/\//, "").replace(/\/$/, "");
  }

  var cards = CHANNELS.map(function (c) {
    return (
      '<div class="community-card">' +
        '<div class="community-card__qr">' +
          '<img src="' + qr(c.url) + '" alt="' + c.name + ' QR code" loading="lazy" />' +
        "</div>" +
        '<div class="community-card__name">' + c.name + "</div>" +
        '<a class="community-card__link" href="' + c.url + '" target="_blank" rel="noopener">' +
          shortLabel(c.url) +
        "</a>" +
      "</div>"
    );
  }).join("");

  var overlay = document.createElement("div");
  overlay.className = "community-overlay";
  overlay.setAttribute("role", "dialog");
  overlay.setAttribute("aria-modal", "true");
  overlay.setAttribute("aria-label", "Join the Orca community");
  overlay.innerHTML =
    '<div class="community-modal">' +
      '<button class="community-modal__close" aria-label="Close">\u2715</button>' +
      '<div class="community-modal__eyebrow">A home for builders</div>' +
      '<h2 class="community-modal__title">Join the Community</h2>' +
      '<p class="community-modal__lead">' + LEAD + "</p>" +
      '<div class="community-grid">' + cards + "</div>" +
    "</div>";
  document.body.appendChild(overlay);

  function open() {
    overlay.classList.add("is-open");
    document.body.style.overflow = "hidden";
  }
  function close() {
    overlay.classList.remove("is-open");
    document.body.style.overflow = "";
  }

  document.querySelectorAll("[data-community-open]").forEach(function (el) {
    el.addEventListener("click", function (e) {
      e.preventDefault();
      open();
    });
  });

  overlay.addEventListener("click", function (e) {
    if (e.target === overlay) close();
  });
  overlay.querySelector(".community-modal__close").addEventListener("click", close);
  document.addEventListener("keydown", function (e) {
    if (e.key === "Escape" && overlay.classList.contains("is-open")) close();
  });
})();
