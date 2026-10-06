/* Silent GIF-like playback. Animation attributes live directly in index.html, matching the original portfolio. */
(function () {
  var preference = window.matchMedia('(prefers-reduced-motion: reduce)');
  function updateMotion() {
    document.querySelectorAll('.demo video').forEach(function (video) {
      video.autoplay = !preference.matches;
      if (preference.matches) video.pause();
      else video.play().catch(function () { /* Preserve poster when autoplay is unavailable. */ });
    });
  }
  updateMotion();
  if (preference.addEventListener) preference.addEventListener('change', updateMotion);
})();
