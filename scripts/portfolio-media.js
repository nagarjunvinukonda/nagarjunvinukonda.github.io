/* Silent GIF-like playback. Respect the visitor's reduced-motion preference. */
(function () {
  var preference = window.matchMedia('(prefers-reduced-motion: reduce)');
  function updateMotion() {
    document.querySelectorAll('.demo video').forEach(function (video) {
      video.autoplay = !preference.matches;
      if (preference.matches) video.pause();
      else video.play().catch(function () { /* Preserve the poster if autoplay is unavailable. */ });
    });
  }
  updateMotion();
  if (preference.addEventListener) preference.addEventListener('change', updateMotion);
})();
